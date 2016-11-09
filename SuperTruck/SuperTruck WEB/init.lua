print("Starting...")

---[===[ UART setup:
uart.setup(0, 115200, 8, 0, 1, 1)
print ("\n#UART READY!#\n")
--]===]
-----------------------------------------------------------------

if pcall(function () print("Open config") dofile("config.lc") end) then
   print("Connecting to WIFI...")
   --Configuring WiFi as STATION
   wifi.setmode(wifi.STATION)
   wifi.sta.config(ssid,password) 
   wifi.sta.connect()
   timeout = 0;
   tmr.alarm(1, 1000, 1, function()
	if wifi.sta.getip() == nil then
	  print("IP unavaiable, waiting... " .. timeout)
	  timeout = timeout + 1
	  if timeout >= 30 then
	    file.remove('config.lc')
	    node.restart()
	  end
	else
	  tmr.stop(1)
	  --do your code here:
	  --MAIN:
	    dofile("main.lua")
	    ---[===[
	    ip, nm, gw = wifi.sta.getip()
	    print("IP Info: \nIP Address: ",ip)
	    print("Netmask: ",nm)
	    print("Gateway Addr: ",gw)
	    print('MAC Address: ', wifi.sta.getmac(),'\n')
	    --]===]
      end
   end)
else
   print("Enter configuration mode")
   dofile("wifi_setup.lua")
end