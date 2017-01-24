import { Template } from 'meteor/templating';
import { Meteor } from 'meteor/meteor';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'


Template.SP_mqttLayout.events({
	'click #sendData': function(){
		Meteor.call('publishMessage','SuperTruck/cmd','#sendData;');
		console.log('sendData Button pressed!');
	},
	'click #stopData': function(){
		Meteor.call('publishMessage','SuperTruck/cmd','#stopData;');
		console.log('stopData Button pressed!');
	},
	'click #cleanDB': function(){
		Meteor.call('SP_cleanDB');
	},
	'click #sendMessage': function(){
		var topic = document.getElementById('m_topic').value;
		var message = document.getElementById('m_message').value;
		Meteor.call('publishMessage',topic,message);
	},
	'click #sendLuaMessage': function(){
		var topic = document.getElementById('m_LuaTopic').value;
		var message = document.getElementById('m_LuaMessage').value;
		Meteor.call('publishMessage',topic,message);
	},
	'click #nodeRestart': function(){
		Meteor.call('publishMessage','SuperTruck/cmd/Lua','node.restart()');
	},
	'click #resetUART': function(){
		Meteor.call('publishMessage','SuperTruck/cmd/Lua','resetUART()');
	}
});


Template.SP_mqttLayout.helpers({
	allMessages: function() {
		const selector = {}
		const filter = { 
			sort: { _date: -1 },
			limit: 20
		}
		const messages = SuperTruck_Messages.find(selector, filter).fetch();

		return messages 	
	},
	savedTopic: function(){
		const selector = {
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
		}
		var _x = SuperTruck_Messages.find(selector, filter);
		return _x;
	}
});
