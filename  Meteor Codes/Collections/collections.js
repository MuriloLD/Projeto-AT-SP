import { Mongo } from 'meteor/mongo'

// initializing the collection, used from client and server
const SuperTruck_Messages = new Mongo.Collection("SP_mqttMessages");
const SuperTruck_VarsDB = new Mongo.Collection("SP_Vars");

const AntTruck_Messages = new Mongo.Collection("AT_mqttMessages");
const AntTruck_VarsDB = new Mongo.Collection("AT_Vars");

export {SuperTruck_Messages, SuperTruck_VarsDB, AntTruck_Messages, AntTruck_VarsDB}; 

