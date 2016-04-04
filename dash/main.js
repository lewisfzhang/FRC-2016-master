var webSocket = null;

var model = {
  tables: {},
  charts: [],
};

var MIN_PLOT_UPDATE_TIME_MILLIS = 50;

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
  container.append($("<div> Table: " + tableName + "</div>"));
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
    var field = $("<input type='text'></input>");
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
  var TABLE = "/SmartDashboard";
  var haveBallElement = findElementModelOrNull(TABLE, "have_ball");
  var haveBallValue = haveBallElement != null && haveBallElement.value;
  var haveBallBox = $("#haveBallBox");
  haveBallBox
    .removeClass("booleanBoxFalse booleanBoxTrue")
    .addClass(haveBallValue ? "booleanBoxTrue" : "booleanBoxFalse");
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
