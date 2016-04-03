var webSocket = null;

$(document).ready(function() {
  kickWebSocket();
  setInterval(kickWebSocket, 1000);
});


function kickWebSocket() {
  if (webSocket != null) {
    // already working
    return;
  }
  webSocket = new WebSocket("ws://localhost:8000");
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
  addOrUpdateValue(payloadJson.table, payloadJson.key, payloadJson.value);
}

function addOrUpdateValue(tableName, key, value) {
  var pane = getOrCreateDashboardPane(tableName);
  var textField = getOrCreateTextField(pane, key);
  textField.val(value);
}

var tablePanes = {};
function getOrCreateDashboardPane(tableName) {
  var pane = tablePanes[tableName];
  if (pane != null) {
    return pane;
  }
  pane = $("<div/>");
  pane.append($("<div> Table: " + tableName + "</div>"));
  $("body").append(pane);
  tablePanes[tableName] = pane;
  return pane;
}

function getOrCreateTextField(pane, key) {
  var className = "key_class_" + key;
  var field = pane.find("." + className);
  if (field.length > 0) {
    return field;
  }
  field = $("<input type='text' class='" + className + "'></input>");
  var container = $("<div/>").append(key + ":").append(field);
  pane.append(container);
  return field;
}

