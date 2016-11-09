import { Meteor } from 'meteor/meteor';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'



Template.SP_odometriaLayout.events({
	'click #goTo': function() {
		var _x = document.getElementById("FPC_x").value;		
		var _y = document.getElementById("FPC_y").value;		
		if ( (_x != '') && (_y != '') ) {
			Meteor.call('publishMessage','SuperTruck/cmd','#goTo('+_x+','+_y+');');
			console.log('#goTo('+_x+','+_y+');');
		}
		//clean inputs?
	},
	'click #stop': function(){
		Meteor.call('publishMessage','SuperTruck/cmd','#stopAll;');
		// document.getElementById("FPC_x").value = '';		
		// document.getElementById("FPC_y").value = '';

		console.log('#stopAll;');
	}
});

Template.SP_odometriaLayout.helpers({
	nav_pos_x: function(){
		const selector = {
			_topic: "SuperTruck/Vars/nav_pos_x"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
		}
		var _x = SuperTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	nav_pos_y: function(){
		const selector = {
			_topic: "SuperTruck/Vars/nav_pos_y"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
			// fields: {_message:1},
		}
		var _y = SuperTruck_VarsDB.find(selector, filter);
		return _y; 
	},
	nav_heading: function(){
		const selector = {
			_topic: "SuperTruck/Vars/nav_heading"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
			// fields: {_message:1},
		}
		var _y = SuperTruck_VarsDB.find(selector, filter);
		return _y; 
	}
});