
-- init mqtt client with keepalive timer 600sec
mqttClient = mqtt.Client('ESP8266_AntTruck', 600, "guest", "guest")--chip id: 16430855

mqttClient:on("offline", function(client) print ("offline") end)

-- on publish message receive event
mqttClient:on("message", function(client, topic, data)
  if data ~= nil then
    print(topic..": "..data)
  end
  if topic == 'AntTruck/cmd/Lua' and data ~= nil then
    local cmdExec=loadstring(data)
    local _, err = pcall(cmdExec)
    if err then
      print(err)
      mqttClient:publish('AntTruck/talk','Error: '..err,0,0)
    else
      print('OK!')
      mqttClient:publish('AntTruck/talk','OK!',0,0)
    end
  end
end)

mqttClient:connect(bip, bport, 0, 0,
  function(client) print('MQTT Connected!')
    --Subscribe to All topics:
    if client:subscribe("AntTruck/cmd/#",0,function(client)
      print("subscribed at AntTruck/cmd/#")
      end) 
    then
      client:publish('AntTruck/talk','ESP8266 Conected!',0,0)
      client:publish('AntTruck/talk','Subscribed to AntTruck/cmd/#',0,0)
      client:publish('AntTruck/talk',"IP Info: \nIP Address: "..ip..
	  "\nNetmask: "..nm.."\nGateway Addr: "..gw.."\nMAC Address: "..
	  wifi.sta.getmac()..'\n',0,0)
      -- Change uart after MQTT connected
      dofile('UART_decode.lua')
      print('UART Decode set!')
    end 
   end,
  function(client,reason) print('MQTT Deu ruim: '..reason)
end)

-- Reset UART by MQTT:
function resetUART()
  uart.on("data",_,_,1)
  print('Uart back to normal!')
end
