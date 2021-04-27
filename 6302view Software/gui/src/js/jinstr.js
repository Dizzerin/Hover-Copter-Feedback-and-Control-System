var ui_change = new CustomEvent('ui_change');
var field_built = new CustomEvent('field_built');

//These two lines are used in conjunction with the ui_change event to make sure we don't fire hundreds off in a second for no reason(limits to ten max a second
var ready_to_fire = true;
var isActive = true;

//Use these when laying out colors!
var standard_colors = ["blue","red","green","yellow","purple"];
var mouseX=0;
var mouseY = 0;
document.addEventListener("mousemove",function(e){
        mouseX = e.pageX;
        mouseY = e.pageY;
});

//constants used for plot sizing:
var PLOT_HEIGHT = 200;
var PLOT_WIDTH = 300;


//constants used for WS state machine:
var IDLE = 0;
var BUILDING = 1; 
var UPDATE =2;
var RUNNING = 3;
//system state variable for WS state machine
var system_state = IDLE;

// If buffer gets too big, toss away
var MAX_DATA_BUFFER = 100000;


//data for building up csv logs:
var MAX_CSV_BUFFER = 100000;
var csv_record = false;
var csv_rows = [];  // Values array by rows
var csv_col_headers = []; //headers of outputs in order
var csv_dom; 

//
var user_inputs = []; //array of hooks for toggle and slider objects
var div_list = [];
var unique_counter = 0;
var displayers = []; //array of hooks for plot and numerical reporter objects
var report_count = []; //total number of displays (will have length of modules)
var report_depth = []; //depth of displays

var old_input = [];

var input_uniques = [];
var current_inputs = [];

var ws;

var gui_land = document.getElementById("gui_land"); //where draggables end up!

/* Problems may occur on some browsers/platforms when the window loses focus.
   The following code warns users at the loss of focus.
   Uncomment if you feel window focus does affect your JS. */

// window.onfocus = function () { 
//   console.log("IN FOCUS");
//   isActive = true; 
//   document.body.style.background = "#eeeeee";
// }; 

// window.onblur = function () { 
//   console.log("OUT OF FOCUS");
//   isActive = false; 
//   document.body.style.background = "red";
// };

window.onload = function(){
    console.log("here");
    console.log(Cookies);
    var ip =  Cookies.get('ip');
    var port = Cookies.get("port");

    if (ip==undefined){
        ip = "127.0.0.1";
    }
    if (port==undefined){
        port = "6302";
    }
    document.getElementById("ipaddress").value = ip;
    document.getElementById("port").value = port;
    ws = new WebSocket("ws://"+ip+":"+port); //create new websocket to the ip of interest
    ws.binaryType="arraybuffer";
    ws.onopen = function(){
    // Web Socket is connected, send data using send()
      console.log("web socket established");
      ws.send("\n"); 
    }; 
    ws.onmessage = function (evt) {
        MessageParser(evt);
    };

    ws.onclose = function(){ 
      // websocket is closed.
      console.log("Connection is closed..."); 
    };  

    document.addEventListener("ui_change", inputEmit);
}

var inputEmit = function(e){
    var t = e.detail["message"];
    console.log(t);
    var u = t.split(":");
    //lookup the "input" id rather than use the input/output id 
    var unique = input_uniques.indexOf(parseInt(u[0]));
    current_inputs[unique]=parseFloat(u[1]);
    ws.send(String(unique)+":"+u[1]+"\n");
}

document.getElementById("ipportsubmit").addEventListener("mousedown",function(){
    var ip = document.getElementById("ipaddress").value; //collect the ip address
    var port = document.getElementById("port").value;
    //Should add in some checker/an alert that comes up if you can't actually connect.
    Cookies.set("ip",ip,{expiry : new Date(2030, 0, 1)});
    Cookies.set("port",port,{expiry : new Date(2030, 0, 1)});
    console.log("connecting...");
    ws = new WebSocket("ws://"+ip+":"+port); //create new websocket to the ip of interest
    ws.binaryType="arraybuffer";
    ws.onopen = function(){
    // Web Socket is connected, send data using send()
     ws.send("\n"); 
      //ws.send(buf); 
    }; 
    ws.onmessage = function (evt) {
        MessageParser(evt);
    };

    ws.onclose = function(){ ///some sort of scope issue
      // websocket is closed.
     console.log("Connection is closed..."); 
    };  
});


document.getElementById("csv_enable").addEventListener("change",function() {
    if (document.getElementById("csv_enable").checked){
        csv_record = true;
    }else{ //when you shut it off
        csv_record = false;
        var nameo = document.getElementById("csv_name").value+"_"+String(Date.now())+".csv"; //bingo was his nameo
        exportCSV(nameo, csv_col_headers, csv_rows);
    }
});

var WipeGUI = function(){
    var len = div_list.length;
    for (var i = 0; i<len; i++){
        var to_junk = div_list.pop();
        to_junk.remove();
    }
    for (var i=0; i<user_inputs.length;i++){
        var to_junk = user_inputs.pop();
        delete to_junk;
    }
    for (var i=0; i<displayers.length;i++){
        var to_junk = displayers.pop();
        delete to_junk;
    }
};

// Generic helper functions
var array_equals = function (array1,array2) {
    // if the other array is a falsy value, return
    if (!array1||!array2)
        return false;

    // compare lengths - can save a lot of time 
    if (array1.length != array2.length)
        return false;

    for (var i = 0, l=array1.length; i < l; i++) {
        if (array1[i] instanceof Array && array2[i] instanceof Array) {
            if(!array_equals(array1[i],array2[i])){
                return false;       
            }
        }           
        else if (array1[i] != array2[i]) { 
            return false;   
        }           
    }       
    return true;
}

// Reshape single delimited list into array of strings.
var reshapeDelim = function(inArray, delimiter) {
    var gaps = [];
    if(inArray[0] == delimiter) inArray = inArray.slice(1); // strip lead
    for (var i = 0; i < inArray.length; i++){
        if (inArray[i]==delimiter) gaps.push(i);  // find all delim
    }
    var dArray = [];
    dArray.push(String.fromCharCode.apply(null,inArray.slice(0,gaps[0])));
    for (var i=0; i<gaps.length-1; i++){
        dArray.push(String.fromCharCode.apply(null,inArray.slice(gaps[i]+1,gaps[i+1])));
    }
    return dArray;
}

// Merge array buffers 
var mergeArrBuf = function(AB1, AB2) {
    var mergedBuf = new ArrayBuffer(AB1.byteLength + AB2.byteLength);
    var mergedBufInt = new Uint8Array(mergedBuf);
    mergedBufInt.set(new Uint8Array(AB1));
    mergedBufInt.set((new Uint8Array(AB2)), AB1.byteLength);
    return mergedBuf;
};

// Find build string in Uint8Array, the "\fB" character pair.
var isBldStrt = function(e,index,dataArr) {
    return((e == 12) && (dataArr[index+1] == 66));
}
// Find build string end in Uint8Array, the "\r\n" character pair.
var isBldEnd = function(e,index,dataArr) {
    return((e == 13) && (dataArr[index+1] == 10));
}

// Find data string in Uint8Array, the "\fR" character pair.
var isDataStrt = function(e,index,dataArr) {
    return((e == 12) && (dataArr[index+1] == 82));
}

var tDataSave = new ArrayBuffer(4);
var plot_buffer = [];

var MessageParser = function(evt) {
    // Merge new packet with what's left of the last packet, and make int view
    var tData = mergeArrBuf(tDataSave, evt.data);
    var tDataB = new Uint8Array(tData);  // Need Int view for comparison
    //console.log(tDataB);
    var startNext = 0;

    // If packet has a build string, \fB, process.
    var endInd = -1;
    var startInd = tDataB.findIndex(isBldStrt);
    if(startInd >= 0) { // We've got a build string.

        startNext = startInd;
        var endInd = startNext + tDataB.slice(startNext).findIndex(isBldEnd); 
        if (endInd > startInd) { //full build frame!
            startNext = endInd;
            var msgStart = startInd+2;
            var msgSize = endInd - msgStart+1;
            MPBuild(tDataB.slice(msgStart,msgStart+msgSize));
            plot_buffer = [];
            for (let i = 0; i < displayers.length; i++) {
                for (let j = 0; j < report_count[i];j++) {
                plot_buffer.push([])
                }
            }
        }
    }
    // If packet has data strings, \fR's, find all complete ones and send.
    if((plot_buffer != []) && (plot_buffer.length > 0)) {  // Data String!
        var pltPts = false;
        var msgSz = 4*plot_buffer.length;
        while(true) {
            startInd = startNext+tDataB.slice(startNext).findIndex(isDataStrt);
            if (startInd >= 0) {
                startNext = startInd;
                let msgSt = startInd + 2;
                let msgEnd = msgSt + msgSz;
                if (tDataB[msgEnd] == 10) { // Ends in \n, Full frame!
                    startNext = msgEnd;
                    fData = new Float32Array(tData.slice(msgSt,msgEnd));
                    var temp = Array.prototype.slice.call(fData);
                    if(csv_record && (csv_rows.length < MAX_CSV_BUFFER)) {
                        //console.log(current_inputs+Array.prototype.slice.call(fData));
                        csv_rows.push(current_inputs.concat(temp)); // Record for CSV
                    }
                    pltPts = true;
                    for(let i = 0; i < plot_buffer.length;  i++) {
                        plot_buffer[i].push(fData[i]);
                    }
                } else{
                    break;
                }
            } else {
                break;
            }
        }
        if (pltPts) {  // Plot and clear buffer
            MPData(plot_buffer);
            for(let i = 0; i < plot_buffer.length;  i++) {
                plot_buffer[i] = [];
            }
        }
    }

    // Save any leftovers for next time
    if(tDataSave.byteLength > MAX_DATA_BUFFER) { // Buffer blown, reset hard.
        tDataSave = new ArrayBuffer(4);
    } else {
        tDataSave = tData.slice(startNext);
    }
};

function MPData(fa) {
    var data_count = 0;
    for (var i = 0; i < displayers.length; i++){
        var data = [];
        for (var j = 0; j < report_count[i];j++){
            data.push([]);
            for (var k = 0; k < fa[data_count].length; k++) {
                data[j].push(fa[data_count][k]);
            }
        data_count+=1;
        }
        displayers[i].step(data);
    }
};

function MPBuild(intData) {
    system_state = BUILDING;
    user_inputs = [];
    csv_col_headers = [];
    csv_rows = [];
    displayers = [];
    report_count = [];
    unique_counter = 0;
    current_inputs = [];
    input_uniques = [];
    WipeGUI();
    var build_array = reshapeDelim(intData, 13); // ~  delim
    console.log(build_array);
    for (var i = 0; i < build_array.length;){
        console.log(i);
        let newdiv = document.createElement("div"); //new div
        newdiv.setAttribute("id","box_"+String(unique_counter)); 
        div_list.push(newdiv); //push to div list (for DOM management
        newdiv.setAttribute("class","cp-item");
        gui_land.appendChild(newdiv);
        let whichFrob = build_array[i];
        switch (whichFrob){ // 
            case "S": //slider
                console.log("building slider");
                var title = build_array[i+1];
                var low = parseFloat(build_array[i+2]);
                var high = parseFloat(build_array[i+3]);
                var res = parseFloat(build_array[i+4]);
                var toggle = build_array[i+5]==="True"?true:false;
                csv_col_headers.push(title);
                current_inputs.push(0);
                input_uniques.push(unique_counter);
                user_inputs.push(new Slider(unique_counter,title,low,high,res,toggle));
                i+=6;
                break;
            case "T": //toggle
                console.log("building toggle");
                var title = build_array[i+1];
                csv_col_headers.push(title);
                current_inputs.push(0);
                input_uniques.push(unique_counter);
                user_inputs.push(new Toggle(unique_counter,title));
                i+=2;
                break;
            case "B": //button
                console.log("building button");
                var title = build_array[i+1];
                csv_col_headers.push(title);
                current_inputs.push(0);
                input_uniques.push(unique_counter);
                user_inputs.push(new Button(unique_counter,title));
                i+=2;
                break;
            case "J": //joystick (NO  CLUE ON HOW TO FORMAT THIS ONE JDS7/30/2019)
                var title = build_array[i+1];
                var low = parseFloat(build_array[i+2]);
                var high = parseFloat(build_array[i+3]);
                var res = parseFloat(build_array[i+4]);
                var toggle = build_array[i+5]==="1"?true:false;
                csv_col_headers.push(title);
                current_inputs.push(0);
                input_uniques.push(unique_counter);
                user_inputs.push(new Joystick(unique_counter,title,low,high,res,toggle));
                i+=6;
                break;
            case "P": //plot:
                console.log("building plot");
                var title = build_array[i+1];
                var v_low = parseFloat(build_array[i+2]);
                var v_high = parseFloat(build_array[i+3]);
                var h_count = parseInt(build_array[i+4]);
                var trace_depth = parseInt(build_array[i+5]); //need to change
                var trace_count = parseFloat(build_array[i+6]);
                report_count.push(trace_count);
                report_depth.push(trace_depth);
                if (trace_count ===1){
                    displayers.push(new Time_Series(unique_counter,title,PLOT_WIDTH,PLOT_HEIGHT,h_count,[v_low,v_high],1,["blue"]));
                    csv_col_headers.push(title);
                }else{
                    var colors = standard_colors.slice(0,trace_count+1);
                    displayers.push(new Time_Series(unique_counter,title,PLOT_WIDTH,PLOT_HEIGHT,h_count,[v_low,v_high],trace_count,colors));
                    for (var j = 0; j<trace_count;j++) csv_col_headers.push(title+"_"+String(j));
                }
                i+=7;
                break;
            case "N": //numerical reporter:
                console.log("building numerical reporter");
                var title = build_array[i+1];
                var depth = parseInt(build_array[i+2]); //need to change
                var type = build_array[i+3];
                report_count.push(1);
                report_depth.push(depth);
                displayers.push(new Numerical_Reporter(unique_counter,title,type,"red","black"));
                csv_col_headers.push(title);
                i+=4;
                break;
            case "#": //starter values
                console.log("found starter");
                var len = user_inputs.length;
                var k;
                console.log(user_inputs);
                for (k=0; k<len; k++){
                    console.log(user_inputs[k]);
                    user_inputs[k].update(build_array[i+k+1]);
                }
                i+=k+1;
                break;
            default:
                i = build_array.length; // Bad String, stop building and abort
        }
        unique_counter += 1;
    }
    document.dispatchEvent(field_built);
};

/* Based off of code from here:
https://stackoverflow.com/questions/14964035/how-to-export-javascript-array-info-to-csv-on-client-side
*/
var exportCSV = function(filename, headers, rows) {

    console.log(headers);
    console.log(rows[0]);
    console.assert(headers.length == rows[0].length,'row header mismatch');
    var csvFile = '';
    
    csvFile = headers[0] + ',';
    for (let i = 1; i < headers.length-1; i++) {
    csvFile += headers[i] + ',';
    }
    csvFile += headers[headers.length-1] + '\n';
    
    for (let i = 0; i < rows.length; i++) {
    for(let j = 0; j < rows[i].length-1; j++) {
            csvFile += rows[i][j] + ',';
    }
    csvFile += rows[i][rows[i].length-1] + '\n';
    }

    var blob = new Blob([csvFile], { type: 'text/csv;charset=utf-8;' });
    if (navigator.msSaveBlob) { // IE 10+
        navigator.msSaveBlob(blob, filename);
    } else {
        var link = document.createElement("a");
        if (link.download !== undefined) { // feature detection
            // Browsers that support HTML5 download attribute
            var url = URL.createObjectURL(blob);
            link.setAttribute("href", url);
            link.setAttribute("download", filename);
            link.style.visibility = 'hidden';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
        }
    }
}



var pckry;
var draggies = [];
var isDrag = false;

//window.addEventListener("load", function(){
document.addEventListener("field_built",function(){
  pckry = new Packery( '.cp', {
    itemSelector: '.cp-item',
    columnWidth:1
  });
  // collection of Draggabillies
  pckry.getItemElements().forEach( function( itemElem ) {
    var draggie = new Draggabilly( itemElem ,{handle:'.handle'});
    draggies.push(draggie);
    pckry.bindDraggabillyEvents( draggie );
    draggie['disable']();
  });

});


// https://stackoverflow.com/questions/27078285/simple-throttle-in-js
function throttle(func, wait, options) {
   var context, args, result;
   var timeout = null;
   var previous = 0;
   if (!options) options = {};
   var later = function() {
     previous = options.leading === false ? 0 : Date.now();
     timeout = null;
     result = func.apply(context, args);
     if (!timeout) context = args = null;
   };
   return function() {
     var now = Date.now();
     if (!previous && options.leading === false) previous = now;
     var remaining = wait - (now - previous);
     context = this;
     args = arguments;
     if (remaining <= 0 || remaining > wait) {
       if (timeout) {
         clearTimeout(timeout);
         timeout = null;
       }
       previous = now;
       result = func.apply(context, args);
       if (!timeout) context = args = null;
     } else if (!timeout && options.trailing !== false) {
       timeout = setTimeout(later, remaining);
     }
     return result;
   };
};


document.getElementById("grid_lock").addEventListener("change",function() {
    // check if checkbox is checked
    var method = isDrag ? 'disable' : 'enable';
    draggies.forEach( function( draggie ) {
        draggie[ method ]();
    });
    // switch flag
    isDrag = !isDrag;
    if (isDrag){
        document.getElementById("grid_status").innerHTML = "Grid UnLocked";
    }else{
        document.getElementById("grid_status").innerHTML = "Grid Locked";
    }
    /*if (document.querySelector('#my-checkbox').checked) {
      // if checked
      console.log('checked');
    } else {
      // if unchecked
      console.log('unchecked');
    }*/
  });

