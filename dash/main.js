var webSocket = null;

var model = {
  tables: {},
  curAutoOptions: [],
};

var MIN_PLOT_UPDATE_TIME_MILLIS = 50;
var TABLE = "/SmartDashboard";

var SELECTED_AUTO_MODE_KEY = "selected_auto_mode";
var SELECTED_AUTO_LANE_KEY = "selected_auto_lane";
var HOOD_TUNING_MODE_KEY = "Hood Tuning Mode";
var COLOR_BOX_COLOR_KEY = "color_box_color";
var COLOR_BOX_TEXT_KEY = "color_box_text";

var TYPE_STRING = "string";
var TYPE_BOOL = "bool";

$(document).ready(function() {

  initAutoLaneButton($("#autoLane1"), "1");
  initAutoLaneButton($("#autoLane2"), "2");
  initAutoLaneButton($("#autoLane3"), "3");
  initAutoLaneButton($("#autoLane4"), "4");
  initAutoLaneButton($("#autoLane5"), "5");

  $("#enableHoodTuningMode").click(function() { setHoodTuningMode(true); });
  $("#disableHoodTuningMode").click(function() { setHoodTuningMode(false); });
  
  kickWebSocket();
  setInterval(kickWebSocket, 1000);
});

function initAutoLaneButton(autoLaneButton, stringValue) {
  autoLaneButton.click(function() {
    sendValueUpdate(TABLE, SELECTED_AUTO_LANE_KEY, stringValue, TYPE_STRING);
  });
}

function setHoodTuningMode(enableHoodTuning) {
  sendValueUpdate(TABLE, HOOD_TUNING_MODE_KEY, enableHoodTuning, TYPE_BOOL);
}

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
  var table = getOrCreateTable(tableName);
  setOrCreateElement(table, key, value);
  refreshDriverStatusElements();
}

function getOrCreateTable(tableName) {
  var table = model.tables[tableName];
  if (table != null) {
    return table;
  }
  // Make a new container
  var container = $("<div/>");
  container.append($("<h3> Table: " + tableName + "</h3>"));
  $("#rawValuesContainer").append(container);
  var pane = $("<div/>");
  container.append(pane);
  
  // make a new model entry
  table = {
    elements: [],
    pane: pane,
    name: tableName,
  };
  model.tables[tableName] = table;
  return table;
}

function setOrCreateElement(table, key, value) {
  var element = null;
  for (var i = 0; i < table.elements.length; i++) {
    if (table.elements[i].key == key) {
      element = table.elements[i];
      break;
    }
  }
  
  if (element == null) {
    // Make a new container
    var field = $("<input type='text' disabled></input>");
    var button = $("<button type='button'>Chart</button>");
    button.click(function() {
      openChartForKey(table.name, key);
    });
    var container = $("<div/>")
        .append("<span style='display: inline-block; width: 200px;'>" + key + ":</span>")
        .append(field)
        .append(button);
    // Make a new model
    element = {
      key: key,
      value: value,
      field: field,
      container: container,
    };
    // Insert the element alphabetically
    insertElementToTable(table, element);
  }

  element.value = value;
  element.field.val(value);
}

function insertElementToTable(table, element) {
  var newKey = element.key;
  for (var i = 0; i < table.elements.length; i++) {
    if (newKey > table.elements[i].key) {
      continue;
    }
    element.container.insertBefore(table.elements[i].container);
    table.elements.splice(i, 0, element);
    return;
  }
  table.pane.append(element.container);
  table.elements.push(element);
}

function openChartForKey(tableName, key) {
  window.open(
    "chart.html?table=" + encodeURIComponent(tableName)
      + "&key=" + encodeURIComponent(key)
      + "&history_minutes=4");
}

/**
 * Update for the hard-coded keys which we want to show in the driver UI
 */
function refreshDriverStatusElements() {
  var colorBoxColorElement = findElementModelOrNull(TABLE, COLOR_BOX_COLOR_KEY);
  var colorBoxTextElement = findElementModelOrNull(TABLE, COLOR_BOX_TEXT_KEY);
  $("#colorBox")
    .text(colorBoxTextElement == null ? "UNKNOWN" : colorBoxTextElement.value)
    .css("background-color", colorBoxColorElement == null ? "orange" : colorBoxColorElement.value);

  setBooleanBoxStyle(
    $("#haveBallBox"),
    getBooleanElementValue(TABLE, "have_ball"));

  maybeRefreshAutoOptions();
  refreshAutoMode();

  var autoLaneElement = findElementModelOrNull(TABLE, SELECTED_AUTO_LANE_KEY);
  $("#selectedAutoLane").text(
    autoLaneElement == null ? "UNKNOWN" : ("Lane " + autoLaneElement.value));

  var airPressureElement = findElementModelOrNull(TABLE, "Air Pressure psi");
  $("#airPressureHolder").text(airPressureElement == null ? "UNKNOWN" : airPressureElement.value);

  var hoodTuningModeElement = findElementModelOrNull(TABLE, HOOD_TUNING_MODE_KEY);
  $("#hoodTuningModeHolder").text(
    hoodTuningModeElement == null ? "UNKNOWN" : hoodTuningModeElement.value);
}

function maybeRefreshAutoOptions() {
  var autoOptionsModel = findElementModelOrNull(TABLE, "auto_options");
  var options = autoOptionsModel == null ? [] : JSON.parse(autoOptionsModel.value);

  // See if the arrays are different
  if (arraysEqual(model.curAutoOptions, options)) {
    // no update
    return;
  }

  // Update the UI
  var container = $("#autoSelectorContainer");
  container.empty();
  for (var i = 0; i < options.length; i++) {
    var box = $("<div/>");
    var button = $("<button type='button'>" + options[i] + "</button>");
    (function (mode) {
      button.click(function() {
        sendValueUpdate(TABLE, SELECTED_AUTO_MODE_KEY, mode, TYPE_STRING);
      });
    })(options[i]);

    box.append(button);
    container.append(box);
  }
  // Update the model
  model.curAutoOptions = options;
}

function sendValueUpdate(tableName, key, value, type) {
  if (webSocket == null || webSocket.readyState != WebSocket.OPEN) {
    console.error("Can't send update to " + tableName + ", " + key);
    return;
  }
  webSocket.send(JSON.stringify({"table": tableName, "key": key, "value": value, type: type}));
}

function arraysEqual(a, b) {
  if (a.length != b.length) {
    return false;
  }
  for (var i = 0; i < a.length; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

function refreshAutoMode() {
  var selectedModeElement = findElementModelOrNull(TABLE, SELECTED_AUTO_MODE_KEY);

  $("#selectedAutoMode").text(
    selectedModeElement == null ? "UNKNOWN" : selectedModeElement.value);
}

function getBooleanElementValue(table, key) {
  var elementModel = findElementModelOrNull(table, key);
  return elementModel != null && elementModel.value;
}

function findElementModelOrNull(tableName, key) {
  var table = model.tables[tableName];
  if (table == null) {
    return null;
  }
  for (var i = 0; i < table.elements.length; i++) {
    var element = table.elements[i];
    if (element.key == key) {
      return element;
    }
  }
  return null;
}

function setBooleanBoxStyle(booleanBox, value) {
  booleanBox
    .removeClass("booleanBoxFalse booleanBoxTrue")
    .addClass(value ? "booleanBoxTrue" : "booleanBoxFalse");
}
