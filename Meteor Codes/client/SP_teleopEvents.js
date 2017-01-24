import { Template } from 'meteor/templating';

//Publish into MQTT topic when pressed button

Template.SP_teleopLayout.events({
	'click #upButton': function() {
		Meteor.call('publishMessage','SuperTruck/cmd','#goUp;');
		console.log('upButton pressed!');
	},
	'click #downButton': function() {
		Meteor.call('publishMessage','SuperTruck/cmd','#goDown;');
		console.log('downButton pressed!');
	},
	'click #leftButton': function() {
		Meteor.call('publishMessage','SuperTruck/cmd','#goLeft;');
		console.log('leftButton pressed!');
	},
	'click #rightButton': function() {
		Meteor.call('publishMessage','SuperTruck/cmd','#goRight;');
		console.log('rightButton pressed!');
	},
	'click #pauseButton': function() {
		Meteor.call('publishMessage','SuperTruck/cmd','#stopAll;');
		console.log('pauseButton pressed!');
	},
	'click #sendCM': function(){
		var DRIVING_v = document.getElementById('DRIVING_v').value;
		var DRIVING_w = document.getElementById('DRIVING_w').value;
		var DRIVING_time = document.getElementById('DRIVING_time').value;
		if ((DRIVING_w!='')&&(DRIVING_v!='')&&(DRIVING_time!='')) {
			Meteor.call('publishMessage','SuperTruck/cmd',
				'#driveRobot('+DRIVING_v+','+DRIVING_w+'/'+DRIVING_time+');');
			console.log('#driveRobot('+DRIVING_v+','+DRIVING_w+'/'+DRIVING_time+');');
		}
	}
});