import { Meteor } from 'meteor/meteor';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'


Template.AT_ultrassomLayout.helpers({
	dist_US_Frente: function(){
		const selector = {
			_topic: "AntTruck/Vars/dist_US_Frente"
		}
		const filter = { 
			sort: { _date: -1 },
			// limit: 1
		}
		var _x = AntTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	dist_US_Direito: function(){
		const selector = {
			_topic: "AntTruck/Vars/dist_US_Direito"
		}
		const filter = { 
			sort: { _date: -1 },
			// limit: 1
		}
		var _y = AntTruck_VarsDB.find(selector, filter);
		return _y; 
	},
		dist_US_Esquerdo: function(){
		const selector = {
			_topic: "AntTruck/Vars/dist_US_Esquerdo"
		}
		const filter = { 
			sort: { _date: -1 },
			// limit: 1
		}
		var _x = AntTruck_VarsDB.find(selector, filter);
		return _x; 
	},
	dist_US_servo: function(){
		const selector = {
			_topic: "AntTruck/Vars/dist_US_servo"
		}
		const filter = { 
			sort: { _date: -1 },
			// limit: 1
		}
		var _y = AntTruck_VarsDB.find(selector, filter);
		return _y; 
	}
});