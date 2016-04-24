var webSocket = null;

var TABLE;
var KEY;
var HISTORY_MINUTES;

$(document).ready(function() {
  TABLE = url("?table");
  KEY = url("?key");
  HISTORY_MINUTES = url("?history") || "0";

  kickWebSocket();
  setInterval(kickWebSocket, 1000);
});

function kickWebSocket() {
  if (webSocket != null) {
    // already working
    return;
  }
  webSocket = new WebSocket(
    "ws://localhost:8000/chart/"
      + encodeURIComponent(TABLE) + "/"
      + encodeURIComponent(KEY) + "/"
      + encodeURIComponent(HISTORY_MINUTES) + "/");
  webSocket.onmessage = function(evt) {
    console.log(evt.data);
    handlePayloadFromWebSocket(evt.data);
  };
  webSocket.onclose = function() {
    webSocket = null;
  };
}

function handlePayloadFromWebSocket(payloadString) {
  var payloadJson = JSON.parse(payloadString);
  $("#logSpewHolder").append(payloadString + "<br/>");
}
