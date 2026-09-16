// Run with an UI-only server on 8765 and Chrome --remote-debugging-port=9333.
// Control and config writes are intercepted in an isolated page; no hardware commands run.
import {writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const target = await (await fetch('http://127.0.0.1:9333/json/new?about:blank',{method:'PUT'})).json();
const ws = new WebSocket(target.webSocketDebuggerUrl); await new Promise(r=>ws.addEventListener('open',r,{once:true}));
let seq=0; const waiters=new Map(), errors=[];
ws.addEventListener('message', ({data})=>{const x=JSON.parse(data);if(x.id){const p=waiters.get(x.id);waiters.delete(x.id);x.error?p?.reject(x.error):p?.resolve(x.result);}else if(x.method==='Runtime.exceptionThrown')errors.push(x.params.exceptionDetails);});
function cdp(method,params={}) {return new Promise((resolve,reject)=>{const id=++seq; waiters.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params}));});}
async function evaluate(expression) {const r=await cdp('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true,replMode:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
async function until(expression){for(let i=0;i<80;i++){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,100));}throw new Error('Timed out: '+expression);}
async function screenshot(name){const result=await cdp('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});await writeFile('/tmp/'+name,Buffer.from(result.data,'base64'));}
await cdp('Runtime.enable');await cdp('Page.enable');
await cdp('Emulation.setDeviceMetricsOverride',{width:1440,height:1040,deviceScaleFactor:1,mobile:false});
await cdp('Page.navigate',{url:'http://127.0.0.1:8765/'});
await until('typeof connected !== "undefined" && connected && taskProfiles.length > 0');
await screenshot('h1-task-layout.png');
assert.equal(await evaluate('document.querySelector(".advanced-config").scrollWidth <= document.querySelector(".advanced-config").clientWidth'),true);
console.log('real preview',await evaluate('({tasks:taskProfiles.length,errors:$("task_feedback").textContent,body:document.documentElement.scrollWidth})'));

assert.equal(await evaluate('document.querySelectorAll("[id^=camera_], [id^=vision_]").length'),0);
const base = await (await fetch('http://127.0.0.1:8765/api/config')).json();
const init = `
window.testPosts=[];window.testFailure=false;window.testDisconnected=false;
window.testConfig=${JSON.stringify({...base,run_mode:'navigation',ui_only:false,task_profiles:[{id:'a',name:'迎宾',steps:[{point:'B',gesture:'wave_hand',turn:'ccw'},{point:'A',gesture:'',turn:'cw'}]},{id:'b',name:'巡检',steps:[{point:'C',gesture:'shake_hand',turn:''}]}],selected_task_id:'a'})};
window.testState={route:{success:true,state:'idle'},teaching_route:{},localization:{ready:true,message:'定位稳定'},processes:{navigation:{running:true}},loaded_task:{id:'a',name:'迎宾',memory_route:'B,A',gesture_route:'B:wave_hand',turn_route:'B:ccw,A:cw'}};
window.confirm=()=>true;
const realFetch=window.fetch.bind(window);window.fetch=async(url,options={})=>{
 if(!String(url).startsWith('/api/')) return realFetch(url,options);
 if(window.testDisconnected) throw new TypeError('连接已断开');
 if(options.method==='POST') {
  window.testPosts.push({url,data:JSON.parse(options.body)});
  await new Promise(r=>setTimeout(r,100));
  if(window.testFailure) return new Response(JSON.stringify({success:false,message:'测试保存失败'}));
  if(url==='/api/config') window.testConfig=JSON.parse(options.body);
  return new Response(JSON.stringify({success:true,message:'ok'}));
 }
 if(url==='/api/config')return new Response(JSON.stringify(window.testConfig));
 if(url==='/api/points')return new Response(JSON.stringify({points:['START','A','B','C'].map((name,i)=>({name,position:{x:i,y:i}}))}));
 if(url==='/api/status')return new Response(JSON.stringify(window.testState));
 if(String(url).startsWith('/api/logs'))return new Response(JSON.stringify({entries:[]}));
 throw new Error('Unexpected request: '+url);
};`;
await cdp('Page.addScriptToEvaluateOnNewDocument',{source:init});
await cdp('Page.navigate',{url:'http://127.0.0.1:8765/?test=tasks'});
await until('typeof connected !== "undefined" && connected && taskProfiles.length===2');
assert.equal(await evaluate('payload().memory_route'),'B,A');
assert.equal(await evaluate('payload().turn_route'),'B:ccw,A:cw');
await evaluate('setTurn("B","cw")');
assert.equal(await evaluate('$("start_route_button").disabled'),true);
await evaluate('setTurn("B","ccw")');
assert.equal(await evaluate('$("start_route_button").disabled'),false);
await evaluate('$("task_select").value="b";$("task_select").dispatchEvent(new Event("change"))');
assert.equal(await evaluate('payload().memory_route'),'C');
assert.equal(await evaluate('payload().gesture_route'),'C:shake_hand');
assert.equal(await evaluate('payload().turn_route'),'');
assert.equal(await evaluate('$("start_route_button").disabled'),true);
await evaluate('await startRoute()');
assert.equal(await evaluate('testPosts.length'),0);
await evaluate(String.raw`switchTask("a");document.querySelector("button[aria-label=\"上移 A\"]").click()`);
assert.equal(await evaluate('payload().memory_route'),'A,B');
assert.equal(await evaluate('payload().turn_route'),'A:cw,B:ccw');
await evaluate(String.raw`const action=document.querySelector("select[aria-label=\"A 到达后动作\"]");action.value="shake_hand";action.dispatchEvent(new Event("change"))`);
assert.equal(await evaluate('payload().gesture_route'),'A:shake_hand,B:wave_hand');
await evaluate('switchTask("b")');
assert.equal(await evaluate('payload().gesture_route'),'C:shake_hand');
assert.equal(await evaluate('payload().turn_route'),'');
await evaluate('createTask(true);$("task_name").value="巡检副本";renameTask();setGesture("C","wave_hand");setUse("A",true);setTurn("C","cw")');
assert.equal(await evaluate('payload().memory_route'),'C,A');
assert.equal(await evaluate('taskProfiles.find(t=>t.id==="b").steps[0].gesture'),'shake_hand');
assert.equal(await evaluate('taskProfiles.find(t=>t.id==="b").steps[0].turn'),'');
await evaluate('await saveConfig()');
assert.equal(await evaluate('savedRevision===revision'),true);
assert.equal(await evaluate('testPosts.at(-1).data.task_profiles.length'),3);
await evaluate('await loadConfig();await loadPoints()');
assert.equal(await evaluate('currentTask().name'),'巡检副本');
assert.equal(await evaluate('payload().turn_route'),'C:cw');
await evaluate(String.raw`const turn=document.querySelector('select[aria-label="C 到点转向"]');turn.value="ccw";turn.dispatchEvent(new Event("change"))`);
assert.equal(await evaluate('payload().turn_route'),'C:ccw');
assert.equal(await evaluate('payload().memory_route'),'C,A');
await evaluate('testFailure=true;setUse("B",true);await saveConfig()');
assert.equal(await evaluate('savedRevision===revision'),false);
assert.equal(await evaluate('$("task_feedback").className'),'error');
await evaluate('testFailure=false;deleteTask();createTask();$("task_name").value="迎宾";renameTask()');
assert.equal(await evaluate('$("task_feedback").className'),'error');
assert.equal(await evaluate('currentTask().steps.length'),0);
await evaluate('deleteTask();switchTask("a");await Promise.all([startNavigation(),startNavigation()])');
assert.equal(await evaluate('testPosts.filter(p=>p.url==="/api/navigation/start").length'),1);
assert.equal(await evaluate('testPosts.at(-1).data.memory_route'),'A,B');
assert.equal(await evaluate('testPosts.at(-1).data.turn_route'),'A:cw,B:ccw');
await evaluate('testState.loaded_task={id:"a",name:"迎宾",memory_route:"A,B",gesture_route:"A:shake_hand,B:wave_hand",turn_route:"A:cw,B:ccw"};await refreshStatus();await startRoute()');
assert.equal(await evaluate('testPosts.at(-1).url'),'/api/route/start');
assert.equal(await evaluate('testPosts.at(-1).data.selected_task_id'),'a');
await evaluate('testDisconnected=true;await refreshStatus()');
assert.equal(await evaluate('$("start_route_button").disabled'),true);
await evaluate('testDisconnected=false;await refreshStatus()');
await screenshot('h1-task-interaction.png');
await cdp('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:false});
assert.equal(await evaluate('document.documentElement.scrollWidth <= innerWidth'),true);
await screenshot('h1-task-mobile.png');
assert.equal(errors.length,0,JSON.stringify(errors));
console.log('Passed: task switch/isolation/order/actions, copy/new/rename/delete, save/reload, failed save, launch payload, loaded-task guard, double-click, disconnect, mobile layout. No JS exceptions.');
await cdp('Page.close');
ws.close();
