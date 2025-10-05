package main

import (
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

var broker string = "tcp://192.168.68.121:1883"
var clientID string = "sensors-go-mqtt-client"

var messageHandler mqtt.MessageHandler = func(client mqtt.Client, msg mqtt.Message) {
	fmt.Printf("received message on topic: %s\n", msg.Topic())
	fmt.Printf("message: %s\n", msg.Payload())
}

var connectLostHandler mqtt.ConnectionLostHandler = func(c mqtt.Client, err error) {
	fmt.Printf("Connection lost: %v\n", err)

}

var connectHandler mqtt.OnConnectHandler = func(c mqtt.Client) {
	fmt.Println("connected to MQTT broker")
}

func main() {
	opts := mqtt.NewClientOptions()
	opts.AddBroker(broker)
	opts.SetClientID(clientID)
	opts.SetUsername("iot_device_user")
	opts.SetPassword("___password___")
	opts.SetDefaultPublishHandler(messageHandler)
	opts.SetOnConnectHandler(connectHandler)
	opts.SetConnectionLostHandler(connectLostHandler)
	opts.SetAutoReconnect(true)
	opts.SetKeepAlive(60 * time.Second)
	opts.SetPingTimeout(10 * time.Second)

	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		log.Fatalf("Failed to connect to MQTT broker: %v", token.Error())

	}

	topic := "sensors/bme280/all"
	if token := client.Subscribe(topic, 0, nil); token.Wait() && token.Error() != nil {
		log.Fatalf("Failed to subscribe to topic: %v", token.Error())
	}
	fmt.Printf("Subscribed to topic: %s\n", topic)

	// wait for interrupt signal to gracefully shutdow
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)
	<-sigChan

	// cleanup
	fmt.Println("\nDisconnecting from MQTT broker...")
	client.Disconnect(250)
	fmt.Println("Disconnected")

}
