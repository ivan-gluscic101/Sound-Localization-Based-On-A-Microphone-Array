#include "web_server.h"
#include "microphone_config.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "WEB_SERVER";
static httpd_handle_t server = NULL;

#define MAX_WS_CLIENTS 4
static int ws_fds[MAX_WS_CLIENTS];
static int ws_fd_count = 0;
static SemaphoreHandle_t ws_fds_mutex = NULL;

/* Served 3D WebGL page. STM32 frame: X forward, Y left, Z up.
 * Flat shader, 3 reference planes, free orbit camera (up=+Z). */
static const char *INDEX_HTML =
"<!DOCTYPE html><html><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Sound Localization 3D</title>"
"<style>"
"*{margin:0;padding:0;}body{overflow:hidden;background:#111;}"
"#hud{position:fixed;top:12px;left:12px;color:#fff;font:14px/1.6 monospace;"
"background:rgba(0,0,0,.65);padding:8px 14px;border-radius:6px;pointer-events:none;}"
"#sts{position:fixed;bottom:12px;left:12px;color:#888;font:12px monospace;}"
"#mic-info{position:fixed;top:12px;right:12px;color:#f44;font:11px/1.4 monospace;"
"background:rgba(0,0,0,.65);padding:8px 12px;border-radius:6px;pointer-events:none;"
"max-width:180px;}"
".mic-label{position:fixed;color:#f44;font:bold 13px monospace;background:rgba(0,0,0,0.7);"
"padding:3px 6px;border-radius:3px;pointer-events:none;border:1px solid #f44;}"
"</style></head><body>"
"<div id='hud'>Azimuth: --&deg; | Elevation: --&deg; | Strength: --</div>"
"<div id='ori-hud' style='position:fixed;top:44px;left:12px;color:#9cf;font:13px/1.5 monospace;background:rgba(0,0,0,.6);padding:6px 12px;border-radius:6px;pointer-events:none;'>Roll: --&deg; | Pitch: --&deg;</div>"
"<div id='sts'>Connecting...</div>"
"<div id='mic-info'>Mikrofoni:<br></div>"
"<div id='mic-labels'></div>"
"<script>"

/* WebGL canvas */
"var cv=document.createElement('canvas');"
"document.body.appendChild(cv);"
"var gl=cv.getContext('webgl');"
"function rsz(){cv.width=innerWidth;cv.height=innerHeight;gl.viewport(0,0,cv.width,cv.height);}"
"rsz();window.addEventListener('resize',rsz);"

/* Flat-color shader with alpha */
"var VS='attribute vec3 aPos;uniform mat4 uMVP;void main(){gl_Position=uMVP*vec4(aPos,1.);}';"
"var FS='precision mediump float;uniform vec4 uCol;void main(){gl_FragColor=uCol;}';"
"function mkSh(t,s){var sh=gl.createShader(t);gl.shaderSource(sh,s);gl.compileShader(sh);return sh;}"
"var prg=gl.createProgram();"
"gl.attachShader(prg,mkSh(gl.VERTEX_SHADER,VS));"
"gl.attachShader(prg,mkSh(gl.FRAGMENT_SHADER,FS));"
"gl.linkProgram(prg);gl.useProgram(prg);"
"var aP=gl.getAttribLocation(prg,'aPos');"
"var uMVP_=gl.getUniformLocation(prg,'uMVP');"
"var uC_=gl.getUniformLocation(prg,'uCol');"

/* Matrix math (column-major) */
"function mm(a,b){var c=new Float32Array(16);"
"for(var i=0;i<4;i++)for(var j=0;j<4;j++)for(var k=0;k<4;k++)c[j*4+i]+=a[k*4+i]*b[j*4+k];"
"return c;}"
"function pm(fov,asp,n,f){var t=Math.tan(fov*Math.PI/360),m=new Float32Array(16);"
"m[0]=1/(t*asp);m[5]=1/t;m[10]=(f+n)/(n-f);m[11]=-1;m[14]=2*f*n/(n-f);return m;}"
/* Look-at, world up=+Z (STM32 frame), looking at origin. */
"function lm(ex,ey,ez){"
"var fx=-ex,fy=-ey,fz=-ez,l=Math.sqrt(fx*fx+fy*fy+fz*fz);fx/=l;fy/=l;fz/=l;"
/* right = forward x worldUp(0,0,1) */
"var rx=fy*1-fz*0,ry=fz*0-fx*1,rz=fx*0-fy*0;"
"var l2=Math.sqrt(rx*rx+ry*ry+rz*rz);rx/=l2;ry/=l2;rz/=l2;"
/* up = right x forward */
"var ux=ry*fz-rz*fy,uy=rz*fx-rx*fz,uz=rx*fy-ry*fx;"
"var m=new Float32Array(16);"
"m[0]=rx;m[1]=ux;m[2]=-fx;m[3]=0;"
"m[4]=ry;m[5]=uy;m[6]=-fy;m[7]=0;"
"m[8]=rz;m[9]=uz;m[10]=-fz;m[11]=0;"
"m[12]=-(rx*ex+ry*ey+rz*ez);m[13]=-(ux*ex+uy*ey+uz*ez);m[14]=fx*ex+fy*ey+fz*ez;m[15]=1;"
"return m;}"
"function tm(x,y,z){return new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,x,y,z,1]);}"
"function sm(s){return new Float32Array([s,0,0,0,0,s,0,0,0,0,s,0,0,0,0,1]);}"
"function rmX(a){var c=Math.cos(a),s=Math.sin(a);"
"return new Float32Array([1,0,0,0,0,c,s,0,0,-s,c,0,0,0,0,1]);}"
"function rmY(a){var c=Math.cos(a),s=Math.sin(a);"
"return new Float32Array([c,0,-s,0,0,1,0,0,s,0,c,0,0,0,0,1]);}"
/* rmZ (yaw) removed: yaw unused */
"var ID=new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);"

/* Board orientation (rad) from IMU 0x05. Model matrix = Ry(pitch)*Rx(roll). */
"var gRoll=0,gPitch=0;"
"function boardM(){return mm(rmY(gPitch),rmX(gRoll));}"

/* Geometry: box (36 verts, pos only) */
"function boxGeo(){"
"var v=[],F=["
"[-1,-1,1,1,-1,1,1,1,1,-1,1,1],"
"[1,-1,-1,-1,-1,-1,-1,1,-1,1,1,-1],"
"[1,-1,-1,1,-1,1,1,1,1,1,1,-1],"
"[-1,-1,1,-1,-1,-1,-1,1,-1,-1,1,1],"
"[-1,1,1,1,1,1,1,1,-1,-1,1,-1],"
"[-1,-1,-1,1,-1,-1,1,-1,1,-1,-1,1]];"
"F.forEach(function(q){[[0,1,2],[0,2,3]].forEach(function(t){"
"t.forEach(function(i){v.push(q[i*3],q[i*3+1],q[i*3+2]);});});});"
"return new Float32Array(v);}"

/* Geometry: UV sphere (pos only) */
"function sphGeo(la,lo){"
"var v=[];"
"for(var i=0;i<la;i++){"
"var t0=(i/la-.5)*Math.PI,t1=((i+1)/la-.5)*Math.PI;"
"for(var j=0;j<lo;j++){"
"var p0=j/lo*2*Math.PI,p1=(j+1)/lo*2*Math.PI;"
"var pt=function(t,p){return[Math.cos(t)*Math.cos(p),Math.sin(t),Math.cos(t)*Math.sin(p)];};"
"var a=pt(t0,p0),b=pt(t1,p0),c=pt(t1,p1),d=pt(t0,p1);"
"[a,b,c,a,c,d].forEach(function(x){v.push(x[0],x[1],x[2]);});}}"
"return new Float32Array(v);}"

/* Geometry: quad in XY plane (z=0), rotated for XZ, YZ */
"function quadGeo(s){return new Float32Array("
"[-s,-s,0, s,-s,0, s,s,0, -s,-s,0, s,s,0, -s,s,0]);}"

/* GPU buffers */
"function mkBuf(data){var b=gl.createBuffer();"
"gl.bindBuffer(gl.ARRAY_BUFFER,b);"
"gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);"
"return{b:b,n:data.length/3};}"
"var bBox=mkBuf(boxGeo()),bSph=mkBuf(sphGeo(10,14)),bQuad=mkBuf(quadGeo(12));"

/* Draw call */
"var VP=ID;"
"function drw(buf,M,col){"
"gl.bindBuffer(gl.ARRAY_BUFFER,buf.b);"
"gl.enableVertexAttribArray(aP);"
"gl.vertexAttribPointer(aP,3,gl.FLOAT,false,12,0);"
"gl.uniformMatrix4fv(uMVP_,false,mm(VP,M));"
"gl.uniform4fv(uC_,new Float32Array(col));"
"gl.drawArrays(gl.TRIANGLES,0,buf.n);}"

/* Orbit camera, full vertical range */
"var cAz=0.4,cEl=0.5,cDst=22,drag=false,lx=0,ly=0;"
"var CLAMP=Math.PI/2-0.02;"
"cv.addEventListener('mousedown',function(e){drag=true;lx=e.clientX;ly=e.clientY;});"
"document.addEventListener('mouseup',function(){drag=false;});"
"cv.addEventListener('mousemove',function(e){"
"if(!drag)return;"
"cAz-=(e.clientX-lx)*.009;"
"cEl=Math.max(-CLAMP,Math.min(CLAMP,cEl+(e.clientY-ly)*.009));"
"lx=e.clientX;ly=e.clientY;});"
"cv.addEventListener('wheel',function(e){"
"cDst=Math.max(4,Math.min(80,cDst+e.deltaY*.04));e.preventDefault();},{passive:false});"
"cv.addEventListener('touchstart',function(e){drag=true;lx=e.touches[0].clientX;ly=e.touches[0].clientY;e.preventDefault();},{passive:false});"
"cv.addEventListener('touchend',function(){drag=false;});"
"cv.addEventListener('touchmove',function(e){"
"if(!drag)return;"
"cAz-=(e.touches[0].clientX-lx)*.009;"
"cEl=Math.max(-CLAMP,Math.min(CLAMP,cEl+(e.touches[0].clientY-ly)*.009));"
"lx=e.touches[0].clientX;ly=e.touches[0].clientY;e.preventDefault();},{passive:false});"

/* Sound sphere pool (max 20, 5s fade) */
"var pool=[];"
"var microphones=[];"

"function loadMicrophones(){"
"fetch('/api/microphones')"
".then(r=>r.json())"
".then(d=>{"
"microphones=d.microphones||[];"
"var mi=document.getElementById('mic-info');"
"var html='Mikrofoni:<br>';"
"microphones.forEach(m=>{"
"html+='<span style=\"color:#f44\">'+m.name+'</span>: ('+m.x.toFixed(2)+', '+m.y.toFixed(2)+', '+m.z.toFixed(2)+') cm<br>';"
"});"
"mi.innerHTML=html;"
"console.log('Mikrofoni učitani:',microphones);"
"initMicrophoneLabels();"
"})"
".catch(e=>{console.error('Greška pri učitavanju mikrofona:',e);});"
"}"

"loadMicrophones();"

"var labelContainer=document.getElementById('mic-labels');"

"function initMicrophoneLabels(){"
"for(var m=0;m<microphones.length;m++){"
"var label=document.createElement('div');"
"label.className='mic-label';"
"label.textContent=microphones[m].name;"
"labelContainer.appendChild(label);"
"}"
"}"

"function updateMicrophoneLabels(){"
"if(!microphones.length)return;"
"var labels=labelContainer.querySelectorAll('.mic-label');"
"var BM=boardM();"
"for(var m=0;m<microphones.length;m++){"
"var mx=microphones[m].x/10,my=microphones[m].y/10,mz=microphones[m].z/10;"
/* Mic position in rotated board frame (BM*translation) */
"var vp=mm(VP,mm(BM,new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,mx,my,mz,1])));"
"var sx=vp[12]/vp[15],sy=vp[13]/vp[15],sz=vp[14]/vp[15];"
"var x=(sx+1)*cv.width/2,y=(-sy+1)*cv.height/2;"
"if(labels[m]){labels[m].style.left=Math.round(x)+'px';labels[m].style.top=Math.round(y)+'px';}"
"}"
"}"

/* az/el in STM32 frame (X fwd=0deg, Y left=90deg, Z up=elevation) */
"function addS(az,el){"
"var a=az*Math.PI/180,e=el*Math.PI/180;"
"pool.push({x:8*Math.cos(e)*Math.cos(a),y:8*Math.cos(e)*Math.sin(a),z:8*Math.sin(e),t:Date.now()});"
"while(pool.length>20)pool.shift();}"

/* WebSocket */
"var hud=document.getElementById('hud'),sts=document.getElementById('sts');"
"var oriHud=document.getElementById('ori-hud');"
"var ws=new WebSocket('ws://'+location.host+'/ws');"
"ws.onopen=function(){sts.textContent='Connected';sts.style.color='#0f0';};"
"ws.onerror=function(){sts.textContent='Error';sts.style.color='#ff6';};"
"ws.onclose=function(e){sts.textContent='Disconnected ('+e.code+')';sts.style.color='#f44';};"
"ws.onmessage=function(ev){"
"try{var d=JSON.parse(ev.data);"
"if(d.type==='orient'){"
/* Board orientation: store angles (deg->rad); pitch/roll only */
"gRoll=d.roll*Math.PI/180;gPitch=d.pitch*Math.PI/180;"
"oriHud.innerHTML='Roll: '+d.roll.toFixed(1)+'&deg; | Pitch: '+d.pitch.toFixed(1)+'&deg;';"
"}else{"
/* Sound direction (board-local frame) */
"hud.innerHTML='Azimuth: '+d.azimuth.toFixed(1)+'&deg; | Elevation: '+(d.polar||0).toFixed(1)+'&deg; | Strength: '+d.strength;"
"addS(d.azimuth,d.polar||0);"
"}}catch(e){}};"

/* Render loop */
"gl.enable(gl.DEPTH_TEST);"
"gl.enable(gl.BLEND);"
"gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);"
"var HP=Math.PI/2;"
"function render(){"
"requestAnimationFrame(render);"
"gl.clearColor(.067,.067,.067,1);"
"gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);"
/* Orbit in STM32 frame: Z is vertical, cEl lifts camera along Z */
"var ex=cDst*Math.cos(cEl)*Math.cos(cAz);"
"var ey=cDst*Math.cos(cEl)*Math.sin(cAz);"
"var ez=cDst*Math.sin(cEl);"
"VP=mm(pm(60,cv.width/cv.height,.1,300),lm(ex,ey,ez));"
/* Opaque: head box + sound spheres */
// "drw(bBox,ID,[0,.67,1,1]);"
/* Board model matrix (IMU rotation): mics, spheres and planes rotate with it */
"var BM=boardM();"
/* Draw mics as small red spheres in rotated board frame */
"for(var m=0;m<microphones.length;m++){"
"var mx=microphones[m].x/10,my=microphones[m].y/10,mz=microphones[m].z/10;"
"drw(bSph,mm(BM,mm(tm(mx,my,mz),sm(.3))),[1,.2,.2,1]);"
"}"
"updateMicrophoneLabels();"
"var now=Date.now();"
"for(var i=pool.length-1;i>=0;i--){"
"var age=now-pool[i].t;"
"if(age>5000){pool.splice(i,1);continue;}"
"var f=1-age/5000;"
/* Sound in board-local frame; BM rotates it to world frame */
"drw(bSph,mm(BM,mm(tm(pool[i].x,pool[i].y,pool[i].z),sm(.5))),[f,.13*f,0,1]);}"
/* Transparent planes: XY (z=0), XZ (y=0), YZ (x=0), rotated with board */
"gl.depthMask(false);"
"drw(bQuad,BM,[1,.35,.35,.12]);"                  /* XY plane (z=0) - red */
"drw(bQuad,mm(BM,rmY(HP)),[.35,1,.35,.12]);"      /* XZ plane (y=0) - green */
"drw(bQuad,mm(BM,rmX(HP)),[.35,.55,1,.12]);"      /* YZ plane (x=0) - blue */
"gl.depthMask(true);"
"}"
"render();"
"</script></body></html>";

/* HTTP GET / : serve the 3D visualization page. */
static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Favicon handler: reply empty to avoid 404s. */
static esp_err_t favicon_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

/* GET /api/microphones : microphone locations as JSON. */
static esp_err_t microphones_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");

    const microphone_t *config = microphone_config_get_all();
    int count = microphone_config_get_count();

    /* ~80 bytes per mic + header */
    int buffer_size = 256 + (count * 80);
    char *json = (char *)malloc(buffer_size);
    if (!json) {
        httpd_resp_send(req, "{\"error\":\"Memory allocation failed\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    int offset = 0;
    offset += snprintf(json + offset, buffer_size - offset, "{\"microphones\":[");
    
    for (int i = 0; i < count; i++) {
        offset += snprintf(json + offset, buffer_size - offset,
            "{\"name\":\"%s\",\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
            config[i].name, config[i].x, config[i].y, config[i].z);
        
        if (i < count - 1) {
            offset += snprintf(json + offset, buffer_size - offset, ",");
        }
    }
    
    offset += snprintf(json + offset, buffer_size - offset, "],\"count\":%d}", count);
    
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    
    return ESP_OK;
}

/* WebSocket handler: register client fd on connect, drain frames otherwise. */
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        xSemaphoreTake(ws_fds_mutex, portMAX_DELAY);
        /* Skip if fd already listed (e.g. browser refresh) */
        int already = 0;
        for (int i = 0; i < ws_fd_count; i++) {
            if (ws_fds[i] == fd) { already = 1; break; }
        }
        if (!already && ws_fd_count < MAX_WS_CLIENTS) {
            ws_fds[ws_fd_count++] = fd;
            ESP_LOGI(TAG, "WS klijent spojen, fd=%d (count=%d)", fd, ws_fd_count);
        }
        xSemaphoreGive(ws_fds_mutex);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    if (ws_pkt.len) {
        uint8_t *buf = calloc(1, ws_pkt.len + 1);
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        free(buf);
    }
    return ret;
}

/* Broadcast null-terminated JSON to all WS clients (thread-safe). */
static void ws_broadcast_json(const char *json) {
    if (!server || !ws_fds_mutex) return;

    httpd_ws_frame_t ws_pkt = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_TEXT,
        .payload    = (uint8_t *)json,
        .len        = strlen(json),
    };

    /* Snapshot fd list under lock to keep the lock short while sending */
    int local_fds[MAX_WS_CLIENTS];
    int local_count = 0;
    xSemaphoreTake(ws_fds_mutex, portMAX_DELAY);
    local_count = ws_fd_count;
    memcpy(local_fds, ws_fds, sizeof(int) * local_count);
    xSemaphoreGive(ws_fds_mutex);

    if (local_count == 0) return;

    /* Send to each fd; collect dead ones */
    int dead_fds[MAX_WS_CLIENTS];
    int dead_count = 0;
    for (int i = 0; i < local_count; i++) {
        esp_err_t err = httpd_ws_send_frame_async(server, local_fds[i], &ws_pkt);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Greška pri slanju na fd=%d (err=0x%x), uklanjam", local_fds[i], err);
            dead_fds[dead_count++] = local_fds[i];
        }
    }

    /* Remove dead fds under lock */
    if (dead_count > 0) {
        xSemaphoreTake(ws_fds_mutex, portMAX_DELAY);
        for (int d = 0; d < dead_count; d++) {
            for (int i = 0; i < ws_fd_count; i++) {
                if (ws_fds[i] == dead_fds[d]) {
                    ws_fds[i] = ws_fds[--ws_fd_count];
                    break;
                }
            }
        }
        xSemaphoreGive(ws_fds_mutex);
    }
}

/* Broadcast sound direction (azimuth/polar/strength) as JSON. */
void web_server_send_data(float azimuth, float polar, uint8_t strength) {
    char json[96];
    snprintf(json, sizeof(json),
             "{\"type\":\"sound\",\"azimuth\":%.1f,\"polar\":%.1f,\"strength\":%u}",
             azimuth, polar, strength);
    ws_broadcast_json(json);
}

/* Broadcast board orientation (roll/pitch) as JSON. */
void web_server_send_orientation(float roll, float pitch) {
    char json[96];
    snprintf(json, sizeof(json),
             "{\"type\":\"orient\",\"roll\":%.1f,\"pitch\":%.1f}",
             roll, pitch);
    ws_broadcast_json(json);
}

/* Init HTTP server, register handlers and WebSocket. */
void web_server_init(void) {
    if (!ws_fds_mutex) {
        ws_fds_mutex = xSemaphoreCreateMutex();
        if (!ws_fds_mutex) {
            ESP_LOGE(TAG, "Neuspjelo kreiranje ws_fds_mutex-a");
            return;
        }
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    static const httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
        .user_ctx = NULL,
    };

    static const httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .user_ctx     = NULL,
        .is_websocket = true,
    };

    static const httpd_uri_t favicon_uri = {
        .uri      = "/favicon.ico",
        .method   = HTTP_GET,
        .handler  = favicon_get_handler,
        .user_ctx = NULL,
    };

    static const httpd_uri_t microphones_uri = {
        .uri      = "/api/microphones",
        .method   = HTTP_GET,
        .handler  = microphones_get_handler,
        .user_ctx = NULL,
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &ws_uri);
        httpd_register_uri_handler(server, &favicon_uri);
        httpd_register_uri_handler(server, &microphones_uri);
        ESP_LOGI(TAG, "HTTP server pokrenut na portu 80");
    } else {
        ESP_LOGE(TAG, "Greška pri pokretanju HTTP servera");
    }
}
