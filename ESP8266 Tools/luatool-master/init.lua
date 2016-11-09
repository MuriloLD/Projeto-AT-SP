-- init.lua --

-- Global Variables (Modify for your network)
ssid = "VIROS"
pass = "h@nsv1r0s"
--ssid = "MURILO"
--pass = "102030405060"

-- Configure Wireless Internet
print('\nINICIANDO INIT.LUA!\n')
wifi.setmode(wifi.STATION)
print('set mode=STATION (mode='..wifi.getmode()..')\n')
print('MAC Address: ',wifi.sta.getmac())
print('Chip ID: ',node.chipid())
print('Heap Size: ',node.heap(),'\n')
-- wifi config start
wifi.sta.config(ssid,pass)
print("Conectado na Rede!\n")
-- wifi config end

-- Connect 
tmr.alarm(0, 1000, 1, function()
   if wifi.sta.getip() == nil then
      print("Connecting to AP...\n")
   else
   	  print('\n**Configurações de rede: \n')
      ip, nm, gw=wifi.sta.getip()
      print("IP Info: \nIP Address: ",ip)
      print("Netmask: ",nm)
      print("Gateway Addr: ",gw,'\n')
      tmr.stop(0)
   end
end)

-- Run the main file
dofile("main.lua")
