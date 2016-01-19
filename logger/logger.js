var client;

$(document).ready(function() {
    client = new Paho.MQTT.Client('localhost', 11883, "clienId");

    // set callback handlers
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;

    connect();

    $("div").text("I'm ready!");
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
    logMessageArray = JSON.parse(message.payloadString);
    console.log(logMessageArray);
}
