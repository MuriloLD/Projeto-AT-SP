
// SuperTruck Routes

FlowRouter.route('/',{
	name: 'home',
	action(){
		BlazeLayout.render('MainLayout',{main: 'homeLayout'});
	}
});

FlowRouter.route('/SP_odometria',{
	name: 'odometria',
	action(){
		BlazeLayout.render('MainLayout',{main: 'SP_odometriaLayout'});
	}
});

FlowRouter.route('/SP_velocidade',{
	name: 'velocidade',
	action(){
		BlazeLayout.render('MainLayout',{main: 'SP_velocidadeLayout'});
	}
});

FlowRouter.route('/SP_ultrassom',{
	name: 'ultrassom',
	action(){
		BlazeLayout.render('MainLayout',{main: 'SP_ultrassomLayout'});
	}
});

FlowRouter.route('/SP_teleop',{
	name: 'teleop',
	action(){
		BlazeLayout.render('MainLayout',{main: 'SP_teleopLayout'});
	}
});

FlowRouter.route('/SP_mqtt',{
	name: 'mqtt',
	action(){
		BlazeLayout.render('MainLayout',{main: 'SP_mqttLayout'});
	}
});

//////////////////////////////////////////////////////////////////////


// AntTruck Routes

FlowRouter.route('/',{
	name: 'home',
	action(){
		BlazeLayout.render('MainLayout',{main: 'homeLayout'});
	}
});

FlowRouter.route('/AT_odometria',{
	name: 'odometria',
	action(){
		BlazeLayout.render('MainLayout',{main: 'AT_odometriaLayout'});
	}
});

FlowRouter.route('/AT_velocidade',{
	name: 'velocidade',
	action(){
		BlazeLayout.render('MainLayout',{main: 'AT_velocidadeLayout'});
	}
});

FlowRouter.route('/AT_ultrassom',{
	name: 'ultrassom',
	action(){
		BlazeLayout.render('MainLayout',{main: 'AT_ultrassomLayout'});
	}
});

FlowRouter.route('/AT_teleop',{
	name: 'teleop',
	action(){
		BlazeLayout.render('MainLayout',{main: 'AT_teleopLayout'});
	}
});

FlowRouter.route('/AT_mqtt',{
	name: 'mqtt',
	action(){
		BlazeLayout.render('MainLayout',{main: 'AT_mqttLayout'});
	}
});

//////////////////////////////////////////////////////////////////////