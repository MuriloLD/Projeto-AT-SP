import { Meteor } from 'meteor/meteor';

import { SuperTruck_Messages } from '../Collections/collections'
import { SuperTruck_VarsDB } from '../Collections/collections'

import { AntTruck_Messages } from '../Collections/collections'
import { AntTruck_VarsDB } from '../Collections/collections'

Template.MainLayout.helpers({
	SP_vBat: function(){
		const selector = {
			_topic: "SuperTruck/Vars/vBateria"
		}
		const filter = { 
			sort: { _date: -1 }
		}
		var _vBat = SuperTruck_VarsDB.findOne(selector, filter)._message/10;
		
			if (_vBat<=10) {
				document.getElementById('sp_vbat').style.backgroundColor = "red";
			}else {
				document.getElementById('sp_vbat').style.backgroundColor = "green";
			}

		return _vBat; 
	},
	AT_vBat: function(){
		const selector = {
			_topic: "AntTruck/Vars/vBateria"
		}
		const filter = { 
			sort: { _date: -1 },
		}
		var _vBat = AntTruck_VarsDB.findOne(selector, filter)._message/10;

			if (_vBat<=10) {
				document.getElementById('at_vbat').style.backgroundColor = "red";
			}else {
				document.getElementById('at_vbat').style.backgroundColor = "green";
			}

		return _vBat; 
	},
});