import { Template } from 'meteor/templating';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'


Template.SP_velocidadeLayout.helpers({
	nav_velLinear: function(){
		const selector = {
			_topic: "SuperTruck/Vars/nav_velLinear"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
		}
		var _x = SuperTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	nav_velAngular: function(){
		const selector = {
			_topic: "SuperTruck/Vars/nav_velAngular"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
			// fields: {_message:1},
		}
		var _y = SuperTruck_VarsDB.find(selector, filter);
		return _y; 
	},
	rPID_Setpoint: function(){
		const selector = {
			_topic: "SuperTruck/Vars/rPID_Setpoint"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
		}
		var _x = SuperTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	lPID_Setpoint: function(){
		const selector = {
			_topic: "SuperTruck/Vars/lPID_Setpoint"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
			// fields: {_message:1},
		}
		var _y = SuperTruck_VarsDB.find(selector, filter);
		return _y; 
	},
	rPID_Input: function(){
		const selector = {
			_topic: "SuperTruck/Vars/rPID_Input"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
		}
		var _x = SuperTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	lPID_Input: function(){
		const selector = {
			_topic: "SuperTruck/Vars/lPID_Input"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
			// fields: {_message:1},
		}
		var _y = SuperTruck_VarsDB.find(selector, filter);
		return _y; 
	},
	rPID_Output: function(){
		const selector = {
			_topic: "SuperTruck/Vars/rPID_Output"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
		}
		var _x = SuperTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	lPID_Output: function(){
		const selector = {
			_topic: "SuperTruck/Vars/lPID_Output"
		}
		const filter = { 
			sort: { _date: -1 },
			limit: 1
			// fields: {_message:1},
		}
		var _y = SuperTruck_VarsDB.find(selector, filter);
		return _y; 
	},
});