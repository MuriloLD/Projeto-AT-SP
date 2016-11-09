import { Meteor } from 'meteor/meteor';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'

// some methods called by the client
Meteor.methods({
    // start receiving messages with the set topic-query
    startClient: function() {
        console.log("startClient called");
        mqttClient.subscribe(topicQuery);
    },
    // stop receiving messages
    stopClient: function() {
        console.log("stopClient called");
        mqttClient.unsubscribe(topicQuery);
    },
    // set a new topic query, unsubscribe from the old and subscribe to the new one
    setTopicQuery: function(newTopicQuery) {
        console.log("set new Topic: " + newTopicQuery);
        mqttClient.unsubscribe(topicQuery).subscribe(newTopicQuery);
        topicQuery = newTopicQuery;
    },
    // send the topic query to the caller
    getTopicQuery: function() {
        return topicQuery;
    },
    // publishes a message with a topic to the broker
    publishMessage: function(topic, message) {
        // console.log("message to send: " + topic + ": " + message);
        mqttClient.publish(topic, message, function() {
            // console.log("message sent: " + message);
        });
    },
    getConfigValues: function() {
        return config;
    },
    SP_cleanDB: function(){
        SuperTruck_Messages.remove({});
        SuperTruck_VarsDB.remove({});
    },
    AT_cleanDB: function(){
        AntTruck_Messages.remove({});
    	AntTruck_VarsDB.remove({});
    },
});

// for development purposes, delete the DB on startup, don't collect too much old data
// Meteor.startup(function () {
//     Messages.remove({});
// });