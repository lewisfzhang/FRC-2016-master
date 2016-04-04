var webSocket = null;

var model = {
  tables: {},
  charts: [],
  curAutoOptions: [],
};

var MIN_PLOT_UPDATE_TIME_MILLIS = 50;
var TABLE = "/SmartDashboard";
var SELECTED_AUTO_MODE_KEY = "selected_auto_mode";
var SELECTED_AUTO_LANE_KEY = "selected_auto_lane";

$(document).ready(function() {

  initAutoLaneButton($("#autoLane1"), "1");
  initAutoLaneButton($("#autoLane2"), "2");
  initAutoLaneButton($("#autoLane3"), "3");
  initAutoLaneButton($("#autoLane4"), "4");
  initAutoLaneButton($("#autoLane5"), "5");

  kickWebSocket();
  setInterval(kickWebSocket, 1000);
});

function initAutoLaneButton(autoLaneButton, stringValue) {
  autoLaneButton.click(function() {
    sendValueUpdate(TABLE, SELECTED_AUTO_LANE_KEY, stringValue);
  });
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
  maybeUpdateChart(tableName, key, value);
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
      startChartingKey(table.name, key);
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

function startChartingKey(tableName, key) {
  // See if this chart already exists
  if (findChartModelOrNull(tableName, key) != null) {
    // This chart already exists
    return;
  }
  // create the new UI
  var container = $("<div><h3>" + key + "<h3></div>");
  $("#chartsContainer").append(container);

  var removeButton = $("<button type='button'>Remove</button>");
  removeButton.click(function() {
    stopChartingKey(tableName, key);
  });
  container.append(removeButton);
  
  var dataContainer = $("<div/>");
  container.append(dataContainer);
  
  // Update the model
  var newChart = {
    tableName: tableName,
    key: key,
    points: [],
    container: container,
    dataContainer: dataContainer,
  };
  model.charts.push(newChart);
}

function stopChartingKey(tableName, key) {
  var chart = findChartModelOrNull(tableName, key);
  if (chart == null) {
    console.error("WTF: Trying to remove unknown chart  " + tableName + ", " + key);
    return;
  }
  // Update the model
  model.charts.splice(model.charts.indexOf(chart), 1);
  // Update the UI
  chart.container.remove();
}

function maybeUpdateChart(tableName, key, value) {
  // Look in the model to see if the chart exists
  var chart = findChartModelOrNull(tableName, key);
  if (chart == null) {
    // not charting this key
    return;
  }
  var now = Date.now();
  if (chart.points.length > 0
      && now - chart.points[chart.points.length - 1].time < MIN_PLOT_UPDATE_TIME_MILLIS) {
    // sampling policy ignores this point
    return;
  }
  var newPoint = {
    time: now,
    val: value,
  };
  chart.points.push(newPoint);
  chart.dataContainer.append(" " + JSON.stringify(newPoint) + ",");
}

function findChartModelOrNull(tableName, key) {
  for (var i = 0; i < model.charts.length; i++) {
    var curChart = model.charts[i];
    if (curChart.tableName == tableName && curChart.key == key) {
      return curChart;
    }
  }
  return null;
}

/**
 * Update for the hard-coded keys which we want to show in the driver UI
 */
function refreshDriverStatusElements() {
  setBooleanBoxStyle(
    $("#haveBallBox"),
    getBooleanElementValue(TABLE, "have_ball"));

  var hood = getBooleanElementValue(TABLE, "hood_on_target");
  setBooleanBoxStyle($("#hoodOnTargetBox"), hood);
  var flywheel = getBooleanElementValue(TABLE, "flywheel_on_target");
  setBooleanBoxStyle($("#flywheelOnTargetBox"), flywheel);
  var turret = getBooleanElementValue(TABLE, "turret_on_target");
  setBooleanBoxStyle($("#turretOnTargetBox"), turret);
  // TODO: drive on target

  setBooleanBoxStyle($("#onTargetBox"), hood && flywheel && turret);

  maybeRefreshAutoOptions();
  refreshAutoMode();

  var autoLaneElement = findElementModelOrNull(TABLE, SELECTED_AUTO_LANE_KEY);
  $("#selectedAutoLane").text(
    autoLaneElement == null ? "UNKNOWN" : ("Lane " + autoLaneElement.value));
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
        sendValueUpdate(TABLE, SELECTED_AUTO_MODE_KEY, mode);
      });
    })(options[i]);

    box.append(button);
    container.append(box);
  }
  // Update the model
  model.curAutoOptions = options;
}

function sendValueUpdate(tableName, key, value) {
  if (webSocket == null || webSocket.readyState != WebSocket.OPEN) {
    console.error("Can't send update to " + tableName + ", " + key);
    return;
  }
  webSocket.send(JSON.stringify({"table": tableName, "key": key, "value": value}));
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
