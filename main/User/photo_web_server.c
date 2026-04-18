/**
 * @file photo_web_server.c
 * @brief SD 卡媒体网页浏览模块实现
 *
 * 模块职责：
 *   - 通过 HTTP 服务提供照片和视频浏览网页
 *   - 扫描 TF 卡中各次上电目录下的 photo 和 video 子目录
 *   - 将 JPEG 照片和 MP4 录像提供给浏览器访问
 *   - 网页加载时接收浏览器/电脑时间，用于校正后续媒体文件命名
 *
 * 目录约定：
 *   /sdcard/002_19800106/photo/1980-01-06T00-01-10-134.jpeg
 *   /sdcard/002_19800106/video/1980-01-06T00-01-10-134_0001.mp4
 */

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "media_storage.h"
#include "photo_web_server.h"
#include "tf_card.h"

static const char *TAG = "photo_web";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define PHOTO_WEB_MAX_PATH_LEN 256
#define PHOTO_WEB_FILE_CHUNK_SIZE 2048
#define PHOTO_WEB_RANGE_HEADER_LEN 96
#define PHOTO_WEB_HTTP_HEADER_LEN 512
#define PHOTO_WEB_TIME_QUERY_LEN 96
#define PHOTO_WEB_TIME_VALUE_LEN 24
#define PHOTO_WEB_DELETE_BODY_MAX_LEN 16384
#define PHOTO_WEB_SERVER_STACK_SIZE (8 * 1024)

typedef struct
{
    httpd_handle_t server;
} photo_web_ctx_t;

typedef struct
{
    bool partial;
    uint64_t start;
    uint64_t end;
    uint64_t length;
} photo_web_range_t;

typedef bool (*photo_web_suffix_check_t)(const char *name);

static photo_web_ctx_t s_photo_web;

/* ------------------------------------------------------------------ */
/* 网页内容                                                            */
/* ------------------------------------------------------------------ */
static const char *s_photo_index_html_v6[] = {
    "<!doctype html><html lang='zh-CN'><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>SD 卡媒体浏览</title>"
    "<style>"
    ":root{--bg:#f4efe7;--panel:#fffaf2;--card:#ffffff;--ink:#1b1b1b;--muted:#6e6457;--line:#e7d8c8;--accent:#c65a1e;--danger:#b3312d;}"
    "*{box-sizing:border-box}body{margin:0;font-family:'Segoe UI','Microsoft YaHei',sans-serif;background:radial-gradient(circle at top,#fff8ef 0,#f4efe7 42%,#ece2d6 100%);color:var(--ink);}a{color:inherit;}"
    ".shell{max-width:1260px;margin:0 auto;padding:24px 16px 40px;}.hero{display:flex;flex-wrap:wrap;gap:16px;align-items:flex-end;justify-content:space-between;margin-bottom:18px;}"
    ".title{margin:0;font-size:clamp(28px,5vw,44px);letter-spacing:.04em;}.desc{margin:8px 0 0;color:var(--muted);font-size:14px;max-width:780px;line-height:1.7;}"
    ".toolbar,.sectionTools,.sectionTitleWrap{display:flex;gap:10px;align-items:center;flex-wrap:wrap;}.badge,.count,.miniBadge{padding:10px 14px;border:1px solid var(--line);border-radius:999px;background:rgba(255,250,242,.92);font-size:13px;color:var(--muted);}"
    ".badge.error{color:#8f201b;border-color:#e7b0ad;background:#fff1f0;}button{border:0;border-radius:999px;padding:11px 16px;font-size:14px;font-weight:700;cursor:pointer;transition:transform .18s ease,box-shadow .18s ease,opacity .18s ease;}button:hover:not(:disabled){transform:translateY(-1px)}button:disabled{opacity:.55;cursor:default;transform:none;box-shadow:none}"
    ".primaryBtn{background:linear-gradient(135deg,#db6d2e,#b94a14);color:#fff;box-shadow:0 14px 30px rgba(185,74,20,.18);}.ghostBtn,.foldBtn{background:#fff;color:var(--ink);border:1px solid var(--line);box-shadow:none;}.dangerBtn,.miniDanger{background:linear-gradient(135deg,#cb5141,#a92a24);color:#fff;box-shadow:0 14px 26px rgba(169,42,36,.16);}.foldBtn{padding:8px 14px;font-size:13px;}.miniDanger{padding:7px 12px;font-size:12px;}"
    ".media{display:grid;gap:18px;}.panel{background:rgba(255,250,242,.86);border:1px solid rgba(231,216,200,.92);border-radius:24px;padding:18px;backdrop-filter:blur(8px);box-shadow:0 18px 42px rgba(74,39,16,.08);}.sectionHead{display:flex;justify-content:space-between;align-items:center;gap:14px;flex-wrap:wrap;margin:0 0 14px;}.sectionBody.hidden{display:none;}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));gap:18px;}.card{position:relative;overflow:hidden;border-radius:20px;background:var(--card);border:1px solid #eadccc;display:flex;flex-direction:column;min-height:310px;box-shadow:0 12px 24px rgba(74,39,16,.06);}.card.selected{border-color:#d56a2f;box-shadow:0 0 0 2px rgba(213,106,47,.16),0 16px 28px rgba(74,39,16,.08);}"
    ".cardTop{position:absolute;top:12px;left:12px;display:flex;align-items:center;z-index:2;pointer-events:none;}.pick{display:inline-flex;align-items:center;justify-content:center;width:34px;height:34px;border-radius:50%;background:rgba(255,255,255,.94);border:1px solid rgba(231,216,200,.95);pointer-events:auto;box-shadow:0 10px 20px rgba(74,39,16,.10);}.pick input{margin:0;width:16px;height:16px;accent-color:#c65a1e;}"
    ".thumb{display:block;aspect-ratio:4/3;background:linear-gradient(135deg,#f9dcc0,#f3b183);overflow:hidden;}.thumb img,.thumb video{display:block;width:100%;height:100%;object-fit:cover;transition:transform .25s ease;background:#14110e;}.card:hover .thumb img,.card:hover .thumb video{transform:scale(1.02);}.videoGrid .thumb{background:linear-gradient(135deg,#2d2720,#7f4b26);}"
    ".meta{padding:14px 14px 16px;display:grid;gap:8px;}.name{font-size:13px;font-weight:700;word-break:break-all;line-height:1.6;}.sub{font-size:12px;color:var(--muted);word-break:break-all;line-height:1.6;}.path{font-size:12px;color:#8d7e6f;word-break:break-all;line-height:1.6;}.metaActions{display:flex;justify-content:space-between;align-items:center;gap:12px;}.open{color:var(--accent);text-decoration:none;font-size:12px;font-weight:700;}.empty{grid-column:1/-1;padding:42px 18px;text-align:center;color:var(--muted);border:1px dashed var(--line);border-radius:18px;background:rgba(255,255,255,.65);}"
    "@media (max-width:760px){.shell{padding:18px 12px 32px}.panel{padding:14px}.grid{grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.card{min-height:0}.sectionTools{width:100%;}}@media (max-width:480px){.grid{grid-template-columns:1fr}.toolbar,.sectionTools{align-items:stretch}.toolbar button,.sectionTools button{width:100%;}.badge,.count,.miniBadge{width:100%;text-align:center;}}"
    "</style></head><body><main class='shell'>"
    "<section class='hero'><div><h1 class='title'>SD 卡媒体浏览</h1><p class='desc'>浏览 SD 卡中的照片和视频，支持分区折叠、进入选择模式后批量全选、取消全选和删除选中内容。</p></div>"
    "<div class='toolbar'><div id='status' class='badge'>正在读取媒体列表...</div><div id='overview' class='badge'>照片 0 张 | 视频 0 段 | 已选 0 项</div><button id='reload' class='primaryBtn' type='button'>刷新列表</button><button id='toggleFoldAll' class='ghostBtn' type='button'>全部折叠</button><button id='toggleAllSelection' class='ghostBtn' type='button'>全选</button><button id='clearSelection' class='ghostBtn' type='button'>清空选择</button><button id='deleteSelection' class='dangerBtn' type='button'>删除选中</button></div></section>"
    "<section class='media'><section class='panel'><div class='sectionHead'><div class='sectionTitleWrap'><button id='photoFoldBtn' class='foldBtn' type='button'>收起</button><h2>照片</h2><div id='photoCount' class='count'>0 张</div><div id='photoSelection' class='miniBadge'>未进入选择</div></div><div class='sectionTools'><button id='photoToggleAllBtn' class='ghostBtn' type='button'>选择照片</button><button id='photoDeleteSelectedBtn' class='dangerBtn' type='button'>删除选中</button></div></div><div id='photoBody' class='sectionBody'><div id='photoGrid' class='grid'></div></div></section>"
    "<section class='panel'><div class='sectionHead'><div class='sectionTitleWrap'><button id='videoFoldBtn' class='foldBtn' type='button'>收起</button><h2>视频</h2><div id='videoCount' class='count'>0 段</div><div id='videoSelection' class='miniBadge'>未进入选择</div></div><div class='sectionTools'><button id='videoToggleAllBtn' class='ghostBtn' type='button'>选择视频</button><button id='videoDeleteSelectedBtn' class='dangerBtn' type='button'>删除选中</button></div></div><div id='videoBody' class='sectionBody'><div id='videoGrid' class='grid videoGrid'></div></div></section></section></main>",
    "<script>"
    "const SECTION_KEYS=['photo','video'];"
    "const state={busy:false,photo:{title:'照片',unit:'张',empty:'SD 卡中暂无照片',items:[],selected:new Set(),collapsed:false,selecting:false},video:{title:'视频',unit:'段',empty:'SD 卡中暂无视频',items:[],selected:new Set(),collapsed:false,selecting:false}};"
    "const refs={status:document.getElementById('status'),overview:document.getElementById('overview'),reloadBtn:document.getElementById('reload'),toggleFoldAllBtn:document.getElementById('toggleFoldAll'),toggleAllSelectionBtn:document.getElementById('toggleAllSelection'),clearSelectionBtn:document.getElementById('clearSelection'),deleteSelectionBtn:document.getElementById('deleteSelection'),photo:{body:document.getElementById('photoBody'),grid:document.getElementById('photoGrid'),count:document.getElementById('photoCount'),selection:document.getElementById('photoSelection'),toggleAllBtn:document.getElementById('photoToggleAllBtn'),deleteBtn:document.getElementById('photoDeleteSelectedBtn'),foldBtn:document.getElementById('photoFoldBtn')},video:{body:document.getElementById('videoBody'),grid:document.getElementById('videoGrid'),count:document.getElementById('videoCount'),selection:document.getElementById('videoSelection'),toggleAllBtn:document.getElementById('videoToggleAllBtn'),deleteBtn:document.getElementById('videoDeleteSelectedBtn'),foldBtn:document.getElementById('videoFoldBtn')}};"
    "function formatSize(size){if(!Number.isFinite(size)||size<=0)return '--';if(size<1024)return size+' B';if(size<1024*1024)return(size/1024).toFixed(1)+' KB';return(size/1024/1024).toFixed(2)+' MB';}"
    "function sortItems(items){return items.slice().sort((a,b)=>String(b.path||'').localeCompare(String(a.path||'')));}"
    "function setStatus(text,isError){refs.status.textContent=text;refs.status.classList.toggle('error',!!isError);}"
    "function createEmptyNode(message){const box=document.createElement('div');box.className='empty';box.textContent=message;return box;}"
    "function appendText(parent,cls,text){const node=document.createElement('div');node.className=cls;node.textContent=text;parent.appendChild(node);}"
    "function totalItemCount(){return state.photo.items.length+state.video.items.length;}"
    "function totalSelectedCount(){return state.photo.selected.size+state.video.selected.size;}"
    "function selectionModeItemCount(){let total=0;SECTION_KEYS.forEach(kind=>{if(state[kind].selecting){total+=state[kind].items.length;}});return total;}"
    "function hasSelectionMode(){return SECTION_KEYS.some(kind=>state[kind].selecting);}"
    "function allItemsSelected(){const total=selectionModeItemCount();if(total===0)return false;let selected=0;SECTION_KEYS.forEach(kind=>{if(state[kind].selecting){selected+=state[kind].selected.size;}});return selected===total;}"
    "function allPanelsCollapsed(){return SECTION_KEYS.every(kind=>state[kind].items.length===0||state[kind].collapsed);}"
    "function updateOverview(){refs.overview.textContent='照片 '+state.photo.items.length+' 张 | 视频 '+state.video.items.length+' 段 | 已选 '+totalSelectedCount()+' 项';}"
    "function updateSectionHeader(kind){const info=state[kind];const ref=refs[kind];const total=info.items.length;const selected=info.selected.size;ref.count.textContent=total+' '+info.unit;ref.selection.textContent=info.selecting?('已选 '+selected+' 项'):'未进入选择';ref.toggleAllBtn.textContent=info.selecting?('取消选择'+info.title):('选择'+info.title);ref.toggleAllBtn.disabled=state.busy||total===0;ref.deleteBtn.disabled=state.busy||!info.selecting||selected===0;ref.foldBtn.disabled=total===0;ref.foldBtn.textContent=info.collapsed?'展开':'收起';ref.body.classList.toggle('hidden',info.collapsed);}"
    "function updateToolbar(){const total=totalItemCount();const selected=totalSelectedCount();const selectable=selectionModeItemCount();refs.reloadBtn.disabled=state.busy;refs.toggleFoldAllBtn.disabled=total===0;refs.toggleFoldAllBtn.textContent=allPanelsCollapsed()?'全部展开':'全部折叠';refs.toggleAllSelectionBtn.disabled=state.busy||selectable===0;refs.toggleAllSelectionBtn.textContent=allItemsSelected()?'取消全选':'全选';refs.clearSelectionBtn.disabled=state.busy||selected===0;refs.deleteSelectionBtn.disabled=state.busy||selected===0||!hasSelectionMode();}"
    "function updateActionStates(){updateOverview();updateSectionHeader('photo');updateSectionHeader('video');updateToolbar();}"
    "function setBusy(busy){state.busy=busy;updateActionStates();}"
    "function clampSelection(kind){const valid=new Set(state[kind].items.map(item=>item.path));state[kind].selected=new Set(Array.from(state[kind].selected).filter(path=>valid.has(path)));if(state[kind].items.length===0){state[kind].selecting=false;state[kind].selected.clear();}}"
    "function pauseOtherVideos(current){document.querySelectorAll('video').forEach(video=>{if(video!==current){video.pause();}});}"
    "function createCard(kind,item){const info=state[kind];const selected=info.selected.has(item.path);const card=document.createElement('article');card.className='card'+(selected?' selected':'');if(info.selecting){const top=document.createElement('div');top.className='cardTop';const pick=document.createElement('label');pick.className='pick';const checkbox=document.createElement('input');checkbox.type='checkbox';checkbox.checked=selected;checkbox.disabled=state.busy;checkbox.setAttribute('aria-label','选择媒体文件');checkbox.addEventListener('change',()=>{if(checkbox.checked){info.selected.add(item.path);}else{info.selected.delete(item.path);}card.classList.toggle('selected',checkbox.checked);updateActionStates();});pick.appendChild(checkbox);top.appendChild(pick);card.appendChild(top);}let thumb;if(kind==='photo'){thumb=document.createElement('a');thumb.className='thumb';thumb.href=item.url;thumb.target='_blank';thumb.rel='noreferrer';const img=document.createElement('img');img.loading='lazy';img.decoding='async';img.src=item.url;img.alt=item.name||'photo';thumb.appendChild(img);}else{thumb=document.createElement('div');thumb.className='thumb';const video=document.createElement('video');video.controls=true;video.preload='none';video.src=item.url;video.setAttribute('playsinline','');video.addEventListener('play',()=>pauseOtherVideos(video));thumb.appendChild(video);}const meta=document.createElement('div');meta.className='meta';appendText(meta,'name',item.name||'');appendText(meta,'sub','目录：'+(item.session||''));appendText(meta,'sub','大小：'+formatSize(Number(item.size_bytes)));appendText(meta,'path',item.path||'');const actions=document.createElement('div');actions.className='metaActions';const open=document.createElement('a');open.className='open';open.href=item.url;open.target='_blank';open.rel='noreferrer';open.textContent=(kind==='photo')?'打开图片':'打开视频';actions.appendChild(open);meta.appendChild(actions);card.appendChild(thumb);card.appendChild(meta);return card;}"
    "function renderSection(kind){const info=state[kind];const ref=refs[kind];ref.grid.innerHTML='';if(!info.items.length){ref.grid.appendChild(createEmptyNode(info.empty));return;}info.items.forEach(item=>ref.grid.appendChild(createCard(kind,item)));}"
    "function renderAll(){renderSection('photo');renderSection('video');updateActionStates();}"
    "function clearAllSelections(){if(state.busy)return;state.photo.selected.clear();state.video.selected.clear();renderAll();}"
    "function toggleSectionSelection(kind){const info=state[kind];if(state.busy||!info.items.length)return;info.selecting=!info.selecting;if(!info.selecting){info.selected.clear();}renderAll();}"
    "function toggleAllSelection(){if(state.busy||!selectionModeItemCount())return;if(allItemsSelected()){SECTION_KEYS.forEach(kind=>{if(state[kind].selecting){state[kind].selected.clear();}});}else{SECTION_KEYS.forEach(kind=>{if(state[kind].selecting){state[kind].selected=new Set(state[kind].items.map(item=>item.path));}});}renderAll();}"
    "function toggleSectionFold(kind){if(!state[kind].items.length)return;state[kind].collapsed=!state[kind].collapsed;updateActionStates();}"
    "function toggleAllFolds(){if(!totalItemCount())return;const collapse=!allPanelsCollapsed();SECTION_KEYS.forEach(kind=>{if(state[kind].items.length>0){state[kind].collapsed=collapse;}});updateActionStates();}"
    "async function fetchList(url){const response=await fetch(url,{cache:'no-store'});let data={};try{data=await response.json();}catch(e){}if(!response.ok){throw new Error(data.error||('HTTP '+response.status));}return Array.isArray(data.items)?data.items:[];}"
    "async function syncDeviceTime(){try{await fetch('/api/time?unix_ms='+Date.now(),{cache:'no-store'});}catch(e){}}"
    "async function refreshMediaList(){const result=await Promise.all([fetchList('/api/photos'),fetchList('/api/videos')]);state.photo.items=sortItems(result[0]);state.video.items=sortItems(result[1]);clampSelection('photo');clampSelection('video');renderAll();}"
    "async function requestDelete(paths){const response=await fetch('/api/delete',{method:'POST',cache:'no-store',headers:{'Content-Type':'text/plain; charset=utf-8'},body:paths.join('\\n')});let data={};try{data=await response.json();}catch(e){}if(!response.ok){throw new Error(data.error||('HTTP '+response.status));}return data;}"
    "function buildDeleteBatches(paths){const batches=[];let current=[];let currentLen=0;const maxLen=3000;for(const path of paths){const lineLen=(path?path.length:0)+1;if(current.length>0&&currentLen+lineLen>maxLen){batches.push(current);current=[];currentLen=0;}current.push(path);currentLen+=lineLen;}if(current.length>0){batches.push(current);}return batches;}"
    "async function loadMedia(doneText,doneIsError){setBusy(true);setStatus('正在同步设备时间...',false);try{await syncDeviceTime();setStatus('正在读取媒体列表...',false);await refreshMediaList();if(doneText){setStatus(doneText,!!doneIsError);}else{setStatus('照片 '+state.photo.items.length+' 张，视频 '+state.video.items.length+' 段',false);}}catch(error){state.photo.items=[];state.video.items=[];state.photo.selected.clear();state.video.selected.clear();state.photo.selecting=false;state.video.selecting=false;renderAll();setStatus(error.message||'读取媒体列表失败',true);}finally{setBusy(false);}}"
    "async function deletePaths(paths){const uniquePaths=Array.from(new Set((paths||[]).filter(Boolean)));if(state.busy||!uniquePaths.length)return;const tips=(uniquePaths.length===1)?'确定删除当前文件吗？':'确定删除选中的 '+uniquePaths.length+' 个文件吗？';if(!window.confirm(tips))return;setBusy(true);try{const batches=buildDeleteBatches(uniquePaths);let summary={requested:0,deleted:0,failed:0};for(let i=0;i<batches.length;i++){setStatus('正在删除第 '+(i+1)+' / '+batches.length+' 批...',false);const result=await requestDelete(batches[i]);summary.requested+=Number(result.requested)||batches[i].length;summary.deleted+=Number(result.deleted)||0;summary.failed+=Number(result.failed)||0;}uniquePaths.forEach(path=>{state.photo.selected.delete(path);state.video.selected.delete(path);});setStatus('正在刷新媒体列表...',false);await refreshMediaList();setStatus('删除完成：成功 '+summary.deleted+'，失败 '+summary.failed,summary.failed>0);}catch(error){setStatus(error.message||'删除失败',true);}finally{setBusy(false);}}"
    "refs.reloadBtn.addEventListener('click',()=>loadMedia());refs.toggleFoldAllBtn.addEventListener('click',toggleAllFolds);refs.toggleAllSelectionBtn.addEventListener('click',toggleAllSelection);refs.clearSelectionBtn.addEventListener('click',clearAllSelections);refs.deleteSelectionBtn.addEventListener('click',()=>deletePaths(Array.from(state.photo.selected).concat(Array.from(state.video.selected))));refs.photo.toggleAllBtn.addEventListener('click',()=>toggleSectionSelection('photo'));refs.video.toggleAllBtn.addEventListener('click',()=>toggleSectionSelection('video'));refs.photo.deleteBtn.addEventListener('click',()=>deletePaths(Array.from(state.photo.selected)));refs.video.deleteBtn.addEventListener('click',()=>deletePaths(Array.from(state.video.selected)));refs.photo.foldBtn.addEventListener('click',()=>toggleSectionFold('photo'));refs.video.foldBtn.addEventListener('click',()=>toggleSectionFold('video'));window.addEventListener('DOMContentLoaded',()=>loadMedia());"
    "</script></body></html>",
    NULL,
};

/* ------------------------------------------------------------------ */
/* 内部工具函数                                                        */
/* ------------------------------------------------------------------ */
static esp_err_t photo_web_append_text(char *dst, size_t dst_size,
                                       size_t *offset, const char *text)
{
    size_t text_len;

    if (!dst || dst_size == 0 || !offset || !text)
    {
        return ESP_ERR_INVALID_ARG;
    }

    text_len = strlen(text);
    if (*offset + text_len >= dst_size)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(dst + *offset, text, text_len);
    *offset += text_len;
    dst[*offset] = '\0';
    return ESP_OK;
}

static esp_err_t photo_web_join_path(char *dst, size_t dst_size,
                                     const char *dir, const char *name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !dir || !name)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, dir);
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, name);
    }

    return ret;
}

static esp_err_t photo_web_build_relative_path(char *dst, size_t dst_size,
                                               const char *session, const char *subdir,
                                               const char *file_name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !session || !subdir || !file_name)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, session);
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, subdir);
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, file_name);
    }

    return ret;
}

static esp_err_t photo_web_build_media_uri(char *dst, size_t dst_size,
                                           const char *uri_prefix,
                                           const char *relative_path)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !uri_prefix || !relative_path)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, uri_prefix);
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, relative_path);
    }

    return ret;
}

static bool photo_web_has_jpeg_suffix(const char *name)
{
    size_t len;

    if (!name)
    {
        return false;
    }

    len = strlen(name);
    if (len >= 5 && strcasecmp(name + len - 5, ".jpeg") == 0)
    {
        return true;
    }
    if (len >= 4 && strcasecmp(name + len - 4, ".jpg") == 0)
    {
        return true;
    }

    return false;
}

static bool photo_web_has_mp4_suffix(const char *name)
{
    size_t len;

    if (!name)
    {
        return false;
    }

    len = strlen(name);
    return len >= 4 && strcasecmp(name + len - 4, ".mp4") == 0;
}

static bool photo_web_path_is_safe(const char *subpath)
{
    const char *p;

    if (!subpath || subpath[0] != '/' || subpath[1] == '\0')
    {
        return false;
    }

    if (strstr(subpath, "..") || strchr(subpath, '\\'))
    {
        return false;
    }

    /* 当前媒体命名规则只需要这些字符，拒绝其它字符可避免路径穿越和 URL 编码歧义。 */
    for (p = subpath + 1; *p; p++)
    {
        if ((*p >= '0' && *p <= '9') ||
            (*p >= 'A' && *p <= 'Z') ||
            (*p >= 'a' && *p <= 'z') ||
            *p == '/' || *p == '_' || *p == '-' || *p == '.')
        {
            continue;
        }
        return false;
    }

    return true;
}

static bool photo_web_relative_path_is_safe(const char *relative_path)
{
    const char *p;

    if (!relative_path || relative_path[0] == '\0' || relative_path[0] == '/')
    {
        return false;
    }

    if (strstr(relative_path, "..") || strchr(relative_path, '\\'))
    {
        return false;
    }

    for (p = relative_path; *p; p++)
    {
        if ((*p >= '0' && *p <= '9') ||
            (*p >= 'A' && *p <= 'Z') ||
            (*p >= 'a' && *p <= 'z') ||
            *p == '/' || *p == '_' || *p == '-' || *p == '.')
        {
            continue;
        }
        return false;
    }

    return true;
}

static bool photo_web_is_dir(const char *path)
{
    struct stat st = {0};

    return (stat(path, &st) == 0) && S_ISDIR(st.st_mode);
}

static bool photo_web_is_regular_file(const char *path, struct stat *out_st)
{
    struct stat st = {0};

    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode))
    {
        return false;
    }

    if (out_st)
    {
        *out_st = st;
    }
    return true;
}

static bool photo_web_dir_is_empty(const char *path)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;
    bool is_empty = true;

    if (!path || !photo_web_is_dir(path))
    {
        return false;
    }

    dir = opendir(path);
    if (!dir)
    {
        return false;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }
        is_empty = false;
        break;
    }

    closedir(dir);
    return is_empty;
}

static void photo_web_try_remove_empty_dir(const char *path)
{
    if (!path || !photo_web_dir_is_empty(path))
    {
        return;
    }

    if (remove(path) != 0 && errno != ENOENT)
    {
        ESP_LOGW(TAG, "删除空目录失败: %s, errno=%d", path, errno);
    }
}

static void photo_web_cleanup_empty_session_dirs(const char *relative_path, const char *subdir)
{
    char session_name[64] = {0};
    char session_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char media_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    const char *slash = NULL;
    size_t session_len;

    if (!relative_path || !subdir)
    {
        return;
    }

    slash = strchr(relative_path, '/');
    if (!slash)
    {
        return;
    }

    session_len = (size_t)(slash - relative_path);
    if (session_len == 0U || session_len >= sizeof(session_name))
    {
        return;
    }

    memcpy(session_name, relative_path, session_len);
    session_name[session_len] = '\0';

    if (photo_web_join_path(session_dir, sizeof(session_dir),
                            tf_card_get_mount_point(), session_name) != ESP_OK)
    {
        return;
    }
    if (photo_web_join_path(media_dir, sizeof(media_dir), session_dir, subdir) != ESP_OK)
    {
        return;
    }

    photo_web_try_remove_empty_dir(media_dir);
    photo_web_try_remove_empty_dir(session_dir);
}

static void photo_web_set_no_cache(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
}

static esp_err_t photo_web_send_json_escaped(httpd_req_t *req, const char *text)
{
    char one_char[1];

    if (!text)
    {
        return ESP_OK;
    }

    for (const char *p = text; *p; p++)
    {
        switch (*p)
        {
        case '\\':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\\\"), TAG, "发送 JSON 转义失败");
            break;
        case '"':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\\""), TAG, "发送 JSON 转义失败");
            break;
        case '\n':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\n"), TAG, "发送 JSON 转义失败");
            break;
        case '\r':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\r"), TAG, "发送 JSON 转义失败");
            break;
        case '\t':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\t"), TAG, "发送 JSON 转义失败");
            break;
        default:
            one_char[0] = *p;
            ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(req, one_char, sizeof(one_char)),
                                TAG, "发送 JSON 字符失败");
            break;
        }
    }

    return ESP_OK;
}

static esp_err_t photo_web_send_json_error(httpd_req_t *req,
                                           const char *status, const char *message)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);

    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "{\"error\":\""), TAG, "发送错误 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, message), TAG, "发送错误信息失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\"}"), TAG, "发送错误 JSON 失败");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_send_delete_result(httpd_req_t *req,
                                              uint32_t requested,
                                              uint32_t deleted,
                                              uint32_t failed)
{
    char resp[128];
    int resp_len;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);

    resp_len = snprintf(resp, sizeof(resp),
                        "{\"ok\":%s,\"requested\":%" PRIu32 ",\"deleted\":%" PRIu32 ",\"failed\":%" PRIu32 "}",
                        (failed == 0U) ? "true" : "false",
                        requested, deleted, failed);
    if (resp_len < 0 || resp_len >= (int)sizeof(resp))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return httpd_resp_sendstr(req, resp);
}

static char *photo_web_trim_text(char *text)
{
    char *start = text;
    char *end;

    if (!text)
    {
        return NULL;
    }

    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n')
    {
        start++;
    }

    end = start + strlen(start);
    while (end > start &&
           (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
    {
        end--;
    }
    *end = '\0';
    return start;
}

static esp_err_t photo_web_read_request_body(httpd_req_t *req, char **body_out)
{
    char *body = NULL;
    int total_len;
    int recv_total = 0;

    if (!req || !body_out)
    {
        return ESP_ERR_INVALID_ARG;
    }

    total_len = req->content_len;
    if (total_len <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (total_len > PHOTO_WEB_DELETE_BODY_MAX_LEN)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    body = (char *)malloc((size_t)total_len + 1U);
    if (!body)
    {
        return ESP_ERR_NO_MEM;
    }

    while (recv_total < total_len)
    {
        int recv_len = httpd_req_recv(req, body + recv_total, (size_t)(total_len - recv_total));

        if (recv_len <= 0)
        {
            free(body);
            return ESP_FAIL;
        }
        recv_total += recv_len;
    }

    body[recv_total] = '\0';
    *body_out = body;
    return ESP_OK;
}

static esp_err_t photo_web_delete_media_file(const char *relative_path)
{
    char file_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    const char *subdir = NULL;

    if (!relative_path || !photo_web_relative_path_is_safe(relative_path))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (strstr(relative_path, "/photo/") != NULL && photo_web_has_jpeg_suffix(relative_path))
    {
        subdir = "photo";
    }
    else if (strstr(relative_path, "/video/") != NULL && photo_web_has_mp4_suffix(relative_path))
    {
        subdir = "video";
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(file_path, sizeof(file_path),
                                            tf_card_get_mount_point(), relative_path),
                        TAG, "构建删除路径失败");

    if (!photo_web_is_regular_file(file_path, NULL))
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (remove(file_path) != 0)
    {
        ESP_LOGW(TAG, "删除媒体文件失败: %s, errno=%d", file_path, errno);
        return ESP_FAIL;
    }

    photo_web_cleanup_empty_session_dirs(relative_path, subdir);
    return ESP_OK;
}

static esp_err_t photo_web_send_media_json(httpd_req_t *req, const char *session,
                                           const char *subdir, const char *file_name,
                                           const struct stat *file_st,
                                           const char *uri_prefix, bool *first_item)
{
    char relative_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char public_uri[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char size_text[32] = {0};
    uint64_t file_size = 0;

    if (!req || !session || !subdir || !file_name || !file_st || !uri_prefix || !first_item)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_build_relative_path(relative_path, sizeof(relative_path),
                                                      session, subdir, file_name),
                        TAG, "构建媒体相对路径失败");
    ESP_RETURN_ON_ERROR(photo_web_build_media_uri(public_uri, sizeof(public_uri),
                                                  uri_prefix, relative_path),
                        TAG, "构建媒体访问地址失败");

    if (file_st->st_size > 0)
    {
        file_size = (uint64_t)file_st->st_size;
    }
    snprintf(size_text, sizeof(size_text), "%" PRIu64, file_size);

    if (!*first_item)
    {
        ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, ","), TAG, "发送 JSON 分隔符失败");
    }
    *first_item = false;

    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "{\"name\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, file_name), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"session\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, session), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"path\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, relative_path), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"url\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, public_uri), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"size_bytes\":"), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, size_text), TAG, "发送 JSON 失败");
    return httpd_resp_sendstr_chunk(req, "}");
}

static esp_err_t photo_web_scan_media_session_dir(httpd_req_t *req, const char *session,
                                                  const char *subdir,
                                                  const char *media_name,
                                                  photo_web_suffix_check_t suffix_check,
                                                  const char *uri_prefix,
                                                  bool *first_item)
{
    esp_err_t ret;
    DIR *media_dir_handle = NULL;
    struct dirent *media_entry = NULL;
    char session_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char media_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};

    if (!req || !session || !subdir || !media_name || !suffix_check || !uri_prefix || !first_item)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(session_dir, sizeof(session_dir),
                                            tf_card_get_mount_point(), session),
                        TAG, "构建上电会话目录失败");
    if (!photo_web_is_dir(session_dir))
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(media_dir, sizeof(media_dir), session_dir, subdir),
                        TAG, "构建媒体目录失败");
    if (!photo_web_is_dir(media_dir))
    {
        return ESP_OK;
    }

    media_dir_handle = opendir(media_dir);
    if (!media_dir_handle)
    {
        ESP_LOGW(TAG, "打开%s目录失败: %s, errno=%d", media_name, media_dir, errno);
        return ESP_OK;
    }

    while ((media_entry = readdir(media_dir_handle)) != NULL)
    {
        struct stat file_st = {0};
        char media_path[PHOTO_WEB_MAX_PATH_LEN] = {0};

        if (!suffix_check(media_entry->d_name))
        {
            continue;
        }

        if (photo_web_join_path(media_path, sizeof(media_path),
                                media_dir, media_entry->d_name) != ESP_OK)
        {
            ESP_LOGW(TAG, "%s路径过长，跳过: %s/%s", media_name, media_dir, media_entry->d_name);
            continue;
        }

        if (!photo_web_is_regular_file(media_path, &file_st))
        {
            continue;
        }

        ret = photo_web_send_media_json(req, session, subdir, media_entry->d_name,
                                        &file_st, uri_prefix, first_item);
        if (ret != ESP_OK)
        {
            closedir(media_dir_handle);
            return ret;
        }
    }

    closedir(media_dir_handle);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* HTTP 处理函数                                                       */
/* ------------------------------------------------------------------ */
static esp_err_t photo_web_index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    photo_web_set_no_cache(req);

    for (size_t i = 0; s_photo_index_html_v6[i] != NULL; i++)
    {
        ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, s_photo_index_html_v6[i]),
                            TAG, "发送媒体网页失败");
    }

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_api_media_handler(httpd_req_t *req,
                                             const char *subdir,
                                             const char *media_name,
                                             photo_web_suffix_check_t suffix_check,
                                             const char *uri_prefix)
{
    esp_err_t ret;
    DIR *root_dir = NULL;
    struct dirent *entry = NULL;
    bool first_item = true;

    if (!req || !subdir || !media_name || !suffix_check || !uri_prefix)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tf_card_is_mounted())
    {
        return photo_web_send_json_error(req, "503 Service Unavailable", "TF 卡未挂载");
    }

    root_dir = opendir(tf_card_get_mount_point());
    if (!root_dir)
    {
        ESP_LOGE(TAG, "打开 TF 卡根目录失败: %s, errno=%d", tf_card_get_mount_point(), errno);
        return photo_web_send_json_error(req, "500 Internal Server Error", "无法读取 TF 卡目录");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    ret = httpd_resp_sendstr_chunk(req, "{\"items\":[");
    if (ret != ESP_OK)
    {
        closedir(root_dir);
        return ret;
    }

    while ((entry = readdir(root_dir)) != NULL)
    {
        if (entry->d_name[0] == '.')
        {
            continue;
        }

        ret = photo_web_scan_media_session_dir(req, entry->d_name, subdir, media_name,
                                               suffix_check, uri_prefix, &first_item);
        if (ret != ESP_OK)
        {
            closedir(root_dir);
            return ret;
        }
    }

    closedir(root_dir);
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "]}"), TAG, "发送 JSON 结束失败");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_api_photos_handler(httpd_req_t *req)
{
    return photo_web_api_media_handler(req, "photo", "照片",
                                       photo_web_has_jpeg_suffix, "/photo/");
}

static esp_err_t photo_web_api_videos_handler(httpd_req_t *req)
{
    return photo_web_api_media_handler(req, "video", "视频",
                                       photo_web_has_mp4_suffix, "/video/");
}

static esp_err_t photo_web_api_time_handler(httpd_req_t *req)
{
    char query[PHOTO_WEB_TIME_QUERY_LEN] = {0};
    char unix_ms_text[PHOTO_WEB_TIME_VALUE_LEN] = {0};
    char *end = NULL;
    long long unix_ms = 0;
    size_t query_len;
    esp_err_t ret;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0U || query_len >= sizeof(query))
    {
        return photo_web_send_json_error(req, "400 Bad Request", "缺少时间参数");
    }

    ret = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (ret != ESP_OK ||
        httpd_query_key_value(query, "unix_ms", unix_ms_text, sizeof(unix_ms_text)) != ESP_OK)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "时间参数错误");
    }

    errno = 0;
    unix_ms = strtoll(unix_ms_text, &end, 10);
    if (errno != 0 || end == unix_ms_text || *end != '\0' || unix_ms <= 0)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "时间参数非法");
    }

    ret = media_storage_sync_time_from_unix_ms((int64_t)unix_ms);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "网页时间同步失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return photo_web_send_json_error(req, "400 Bad Request", "设备拒绝该时间");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t photo_web_api_delete_handler(httpd_req_t *req)
{
    char *body = NULL;
    char *cursor = NULL;
    uint32_t requested = 0;
    uint32_t deleted = 0;
    uint32_t failed = 0;
    esp_err_t ret;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tf_card_is_mounted())
    {
        return photo_web_send_json_error(req, "503 Service Unavailable", "TF 卡未挂载");
    }

    ret = photo_web_read_request_body(req, &body);
    if (ret == ESP_ERR_INVALID_ARG)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "删除请求为空");
    }
    if (ret == ESP_ERR_INVALID_SIZE)
    {
        return photo_web_send_json_error(req, "413 Payload Too Large", "删除请求过大");
    }
    if (ret == ESP_ERR_NO_MEM)
    {
        return photo_web_send_json_error(req, "500 Internal Server Error", "内存不足");
    }
    if (ret != ESP_OK)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "删除请求读取失败");
    }

    cursor = body;
    while (cursor && *cursor != '\0')
    {
        char *line_end = strpbrk(cursor, "\r\n");
        char *line = cursor;

        if (line_end)
        {
            *line_end = '\0';
            cursor = line_end + 1;
            while (*cursor == '\r' || *cursor == '\n')
            {
                cursor++;
            }
        }
        else
        {
            cursor = NULL;
        }

        line = photo_web_trim_text(line);
        if (!line || line[0] == '\0')
        {
            continue;
        }

        requested++;
        ret = photo_web_delete_media_file(line);
        if (ret == ESP_OK)
        {
            deleted++;
        }
        else
        {
            failed++;
            ESP_LOGW(TAG, "网页删除失败 | 路径=%s | ret=0x%x (%s)",
                     line, ret, esp_err_to_name(ret));
        }
    }

    free(body);

    if (requested == 0U)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "未提供待删除路径");
    }

    ESP_LOGI(TAG, "网页删除完成 | 请求=%" PRIu32 " | 成功=%" PRIu32 " | 失败=%" PRIu32,
             requested, deleted, failed);
    return photo_web_send_delete_result(req, requested, deleted, failed);
}

static bool photo_web_parse_u64_value(const char **cursor, uint64_t *value)
{
    uint64_t result = 0;
    const char *p;
    bool has_digit = false;

    if (!cursor || !*cursor || !value)
    {
        return false;
    }

    p = *cursor;
    while (*p >= '0' && *p <= '9')
    {
        uint64_t digit = (uint64_t)(*p - '0');

        if (result > (UINT64_MAX - digit) / 10U)
        {
            return false;
        }
        result = result * 10U + digit;
        has_digit = true;
        p++;
    }

    if (!has_digit)
    {
        return false;
    }

    *cursor = p;
    *value = result;
    return true;
}

static esp_err_t photo_web_parse_range_request(httpd_req_t *req, uint64_t file_size,
                                               photo_web_range_t *range)
{
    char range_header[PHOTO_WEB_RANGE_HEADER_LEN] = {0};
    size_t header_len;
    const char *p;

    if (!req || !range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    range->partial = false;
    range->start = 0;
    range->end = (file_size > 0U) ? (file_size - 1U) : 0U;
    range->length = file_size;

    header_len = httpd_req_get_hdr_value_len(req, "Range");
    if (header_len == 0U)
    {
        return ESP_OK;
    }
    if (header_len >= sizeof(range_header))
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (httpd_req_get_hdr_value_str(req, "Range", range_header, sizeof(range_header)) != ESP_OK)
    {
        return ESP_ERR_INVALID_ARG;
    }

    p = range_header;
    if (strncmp(p, "bytes=", strlen("bytes=")) != 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    p += strlen("bytes=");

    if (*p == '-')
    {
        uint64_t suffix_len = 0;

        p++;
        if (!photo_web_parse_u64_value(&p, &suffix_len) || *p != '\0' ||
            suffix_len == 0U || file_size == 0U)
        {
            return ESP_ERR_INVALID_ARG;
        }

        range->start = (suffix_len >= file_size) ? 0U : (file_size - suffix_len);
        range->end = file_size - 1U;
    }
    else
    {
        uint64_t start = 0;
        uint64_t end = 0;

        if (!photo_web_parse_u64_value(&p, &start) || *p != '-')
        {
            return ESP_ERR_INVALID_ARG;
        }
        p++;

        if (*p == '\0')
        {
            if (file_size == 0U || start >= file_size)
            {
                return ESP_ERR_INVALID_ARG;
            }
            end = file_size - 1U;
        }
        else
        {
            if (!photo_web_parse_u64_value(&p, &end) || *p != '\0' ||
                file_size == 0U || start >= file_size || end < start)
            {
                return ESP_ERR_INVALID_ARG;
            }
            if (end >= file_size)
            {
                end = file_size - 1U;
            }
        }

        range->start = start;
        range->end = end;
    }

    range->partial = true;
    range->length = range->end - range->start + 1U;
    return ESP_OK;
}

static void photo_web_close_request_session(httpd_req_t *req)
{
    int sockfd;

    if (!req)
    {
        return;
    }

    sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0)
    {
        httpd_sess_trigger_close(req->handle, sockfd);
    }
}

static esp_err_t photo_web_raw_send_all(httpd_req_t *req, const char *buf, size_t len)
{
    size_t sent_total = 0;

    if (!req || (!buf && len > 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    while (sent_total < len)
    {
        int sent = httpd_send(req, buf + sent_total, len - sent_total);
        if (sent <= 0)
        {
            photo_web_close_request_session(req);
            return ESP_ERR_HTTPD_RESP_SEND;
        }
        sent_total += (size_t)sent;
    }

    return ESP_OK;
}

static esp_err_t photo_web_send_range_error(httpd_req_t *req, uint64_t file_size)
{
    char header[PHOTO_WEB_HTTP_HEADER_LEN];
    int header_len;

    header_len = snprintf(header, sizeof(header),
                          "HTTP/1.1 416 Range Not Satisfiable\r\n"
                          "Content-Type: text/plain\r\n"
                          "Content-Length: 0\r\n"
                          "Accept-Ranges: bytes\r\n"
                          "Content-Range: bytes */%" PRIu64 "\r\n"
                          "Cache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n",
                          file_size);
    if (header_len < 0 || header_len >= (int)sizeof(header))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return photo_web_raw_send_all(req, header, (size_t)header_len);
}

static esp_err_t photo_web_send_video_header(httpd_req_t *req, const photo_web_range_t *range,
                                             uint64_t file_size)
{
    char header[PHOTO_WEB_HTTP_HEADER_LEN];
    int header_len;

    if (!req || !range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (range->partial)
    {
        header_len = snprintf(header, sizeof(header),
                              "HTTP/1.1 206 Partial Content\r\n"
                              "Content-Type: video/mp4\r\n"
                              "Content-Length: %" PRIu64 "\r\n"
                              "Accept-Ranges: bytes\r\n"
                              "Content-Range: bytes %" PRIu64 "-%" PRIu64 "/%" PRIu64 "\r\n"
                              "Cache-Control: no-store\r\n"
                              "Connection: close\r\n"
                              "Content-Disposition: inline\r\n\r\n",
                              range->length, range->start, range->end, file_size);
    }
    else
    {
        header_len = snprintf(header, sizeof(header),
                              "HTTP/1.1 200 OK\r\n"
                              "Content-Type: video/mp4\r\n"
                              "Content-Length: %" PRIu64 "\r\n"
                              "Accept-Ranges: bytes\r\n"
                              "Cache-Control: no-store\r\n"
                              "Connection: close\r\n"
                              "Content-Disposition: inline\r\n\r\n",
                              range->length);
    }

    if (header_len < 0 || header_len >= (int)sizeof(header))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return photo_web_raw_send_all(req, header, (size_t)header_len);
}

static esp_err_t photo_web_send_video_body(httpd_req_t *req, FILE *fp,
                                           const char *file_path,
                                           const photo_web_range_t *range)
{
    char file_buf[PHOTO_WEB_FILE_CHUNK_SIZE];
    uint64_t remain;

    if (!req || !fp || !file_path || !range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (range->start > 0U)
    {
        if (range->start > (uint64_t)LONG_MAX ||
            fseek(fp, (long)range->start, SEEK_SET) != 0)
        {
            ESP_LOGE(TAG, "定位视频文件失败: %s, offset=%" PRIu64 ", errno=%d",
                     file_path, range->start, errno);
            return ESP_FAIL;
        }
    }

    remain = range->length;
    while (remain > 0U)
    {
        size_t want_len = (remain > sizeof(file_buf)) ? sizeof(file_buf) : (size_t)remain;
        size_t read_len = fread(file_buf, 1, want_len, fp);

        if (read_len == 0U)
        {
            ESP_LOGE(TAG, "读取视频文件失败: %s", file_path);
            return ESP_FAIL;
        }

        esp_err_t ret = photo_web_raw_send_all(req, file_buf, read_len);
        if (ret != ESP_OK)
        {
            return ret;
        }
        remain -= (uint64_t)read_len;
    }

    return ESP_OK;
}

static esp_err_t photo_web_photo_handler(httpd_req_t *req)
{
    FILE *fp = NULL;
    struct stat file_st = {0};
    char file_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char file_buf[PHOTO_WEB_FILE_CHUNK_SIZE];
    const char *subpath = req->uri + strlen("/photo");
    size_t offset = 0;
    esp_err_t ret = ESP_OK;

    if (!tf_card_is_mounted())
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF 卡未挂载");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/photo/") == NULL ||
        !photo_web_has_jpeg_suffix(subpath))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "照片路径非法");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                              &offset, tf_card_get_mount_point()),
                        TAG, "构建本地照片路径失败");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "构建本地照片路径失败");

    if (!photo_web_is_regular_file(file_path, &file_st))
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "照片不存在");
    }

    fp = fopen(file_path, "rb");
    if (!fp)
    {
        ESP_LOGE(TAG, "打开照片文件失败: %s, errno=%d", file_path, errno);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "打开照片失败");
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline");

    while (1)
    {
        size_t read_len = fread(file_buf, 1, sizeof(file_buf), fp);
        if (read_len > 0)
        {
            ret = httpd_resp_send_chunk(req, file_buf, read_len);
            if (ret != ESP_OK)
            {
                fclose(fp);
                photo_web_close_request_session(req);
                return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
            }
        }

        if (read_len < sizeof(file_buf))
        {
            if (ferror(fp))
            {
                ESP_LOGE(TAG, "读取照片文件失败: %s", file_path);
                fclose(fp);
                return ESP_FAIL;
            }
            break;
        }
    }

    fclose(fp);
    ret = httpd_resp_send_chunk(req, NULL, 0);
    if (ret != ESP_OK)
    {
        photo_web_close_request_session(req);
    }
    return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
}

static esp_err_t photo_web_video_handler(httpd_req_t *req)
{
    FILE *fp = NULL;
    struct stat file_st = {0};
    char file_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    const char *subpath = req->uri + strlen("/video");
    size_t offset = 0;
    uint64_t file_size = 0;
    photo_web_range_t range = {0};
    esp_err_t ret;
    bool send_body;

    if (!tf_card_is_mounted())
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF 卡未挂载");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/video/") == NULL ||
        !photo_web_has_mp4_suffix(subpath))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "视频路径非法");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                              &offset, tf_card_get_mount_point()),
                        TAG, "构建本地视频路径失败");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "构建本地视频路径失败");

    if (!photo_web_is_regular_file(file_path, &file_st))
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "视频不存在");
    }

    if (file_st.st_size > 0)
    {
        file_size = (uint64_t)file_st.st_size;
    }

    ret = photo_web_parse_range_request(req, file_size, &range);
    if (ret != ESP_OK)
    {
        ret = photo_web_send_range_error(req, file_size);
        photo_web_close_request_session(req);
        return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
    }

    send_body = (req->method != HTTP_HEAD);
    if (send_body)
    {
        fp = fopen(file_path, "rb");
        if (!fp)
        {
            ESP_LOGE(TAG, "打开视频文件失败: %s, errno=%d", file_path, errno);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "打开视频失败");
        }
    }

    ret = photo_web_send_video_header(req, &range, file_size);
    if (ret == ESP_OK && send_body)
    {
        ret = photo_web_send_video_body(req, fp, file_path, &range);
    }

    if (fp)
    {
        fclose(fp);
    }

    photo_web_close_request_session(req);
    return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
}

static esp_err_t photo_web_favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */
esp_err_t photo_web_server_start(void)
{
    esp_err_t ret;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (s_photo_web.server)
    {
        return ESP_OK;
    }

    config.server_port = PHOTO_WEB_SERVER_PORT;
    config.stack_size = PHOTO_WEB_SERVER_STACK_SIZE;
    config.max_open_sockets = 6;
    config.max_uri_handlers = 10;
    config.backlog_conn = 4;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ret = httpd_start(&s_photo_web.server, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "启动媒体网页服务失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = photo_web_index_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_photos_uri = {
        .uri = "/api/photos",
        .method = HTTP_GET,
        .handler = photo_web_api_photos_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_videos_uri = {
        .uri = "/api/videos",
        .method = HTTP_GET,
        .handler = photo_web_api_videos_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_time_uri = {
        .uri = "/api/time",
        .method = HTTP_GET,
        .handler = photo_web_api_time_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_delete_uri = {
        .uri = "/api/delete",
        .method = HTTP_POST,
        .handler = photo_web_api_delete_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t photo_uri = {
        .uri = "/photo/*",
        .method = HTTP_GET,
        .handler = photo_web_photo_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t video_uri = {
        .uri = "/video/*",
        .method = HTTP_GET,
        .handler = photo_web_video_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t video_head_uri = {
        .uri = "/video/*",
        .method = HTTP_HEAD,
        .handler = photo_web_video_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = photo_web_favicon_handler,
        .user_ctx = NULL,
    };

    ret = httpd_register_uri_handler(s_photo_web.server, &index_uri);
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_photos_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_videos_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_time_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_delete_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &photo_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &video_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &video_head_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &favicon_uri);
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "注册媒体网页 URI 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        photo_web_server_stop();
        return ret;
    }

    ESP_LOGI(TAG, "媒体网页服务已启动，端口 %d", PHOTO_WEB_SERVER_PORT);
    return ESP_OK;
}

void photo_web_server_stop(void)
{
    if (!s_photo_web.server)
    {
        return;
    }

    httpd_stop(s_photo_web.server);
    s_photo_web.server = NULL;
    ESP_LOGI(TAG, "媒体网页服务已停止");
}
