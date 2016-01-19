var client;
var logDiv;

$(document).ready(function() {
	logDiv = $("#log_div");
    client = new Paho.MQTT.Client('localhost', 11883, "clienId");

    // set callback handlers
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;

    connect();

    logDiv.text("I'm ready!");
});

function connect() {
	client.connect({onSuccess: onConnect, onFailure: scheduleReconnect});
}

// called when the client connects
function onConnect() {
    // Once a connection has been made, make a subscription and send a message.
    console.log("onConnect");
    //client.subscribe("$SYS/#");
    client.subscribe("/robot_logging");
}

function scheduleReconnect() {
    setTimeout(connect, 1000);
}

// called when the client loses its connection
function onConnectionLost(responseObject) {
    if (responseObject.errorCode !== 0) {
        console.log("onConnectionLost:"+responseObject.errorMessage);
    }
	scheduleReconnect();
}

// called when a message arrives
function onMessageArrived(message) {
    var logMessageArray = JSON.parse(message.payloadString);
	for (var i = 0; i < logMessageArray.length; ++i) {
		logDiv.append($("<div/>").text(JSON.stringify(logMessageArray[i])));
	}
    console.log(logMessageArray);
}
