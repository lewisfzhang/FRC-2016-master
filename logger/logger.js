var client;

$(document).ready(function() {
	client = new Paho.MQTT.Client('localhost', 11883, "clienId");

    // set callback handlers
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;

    // connect the client
    client.connect({onSuccess:onConnect});

	$("div").text("I'm ready!");
});

// called when the client connects
function onConnect() {
    // Once a connection has been made, make a subscription and send a message.
    console.log("onConnect");
    //client.subscribe("$SYS/#");
    client.subscribe("/robot_logging");
}

// called when the client loses its connection
function onConnectionLost(responseObject) {
    if (responseObject.errorCode !== 0) {
		console.log("onConnectionLost:"+responseObject.errorMessage);
    }
}

// called when a message arrives
function onMessageArrived(message) {
	logMessageArray = JSON.parse(message.payloadString);
	console.log(logMessageArray);
}
