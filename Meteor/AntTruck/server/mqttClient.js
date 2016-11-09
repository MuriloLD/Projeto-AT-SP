import { Meteor } from 'meteor/meteor';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'

var mqtt = require('mqtt');

// All messages at topic: SuperTruck/#
    mqttClient = mqtt.connect('mqtt://localhost', {
      port: 1883,
      clientId: 'Meteor Server',
      username: 'ispace',
      password: 'ispace'
    });
      
      
    mqttClient.on('connect', function () {
      mqttClient.subscribe('SuperTruck/#');
      mqttClient.subscribe('AntTruck/#');
      mqttClient.publish('SuperTruck/Server', 'Meteor Connected!');
    });

    mqttClient.on('message', function (topic,message) {
        // console.log('topic: '+topic+' message: '+message);

        if (topic.toString().startsWith('SuperTruck/Vars')) {
            insertMessage(topic,message,SuperTruck_VarsDB);
        } 
        else if(topic.toString().startsWith('AntTruck/Vars')){
            insertMessage(topic,message,AntTruck_VarsDB);
        }
        else if(topic.toString().startsWith('SuperTruck/')){
            insertMessage(topic,message,SuperTruck_Messages);
        }
        else if(topic.toString().startsWith('AntTruck/')){
            insertMessage(topic,message,AntTruck_Messages);
        }
    });
/////////////////////////////////////////////////////////////////

// msg vars must be included in a Meteor Fiber:
var insertMessage = Meteor.bindEnvironment(function(topic,message,DataBase) {
    DataBase.insert({
    	_topic: topic,
		_message: message.toString(),
		_date: new Date(),
    });
});


