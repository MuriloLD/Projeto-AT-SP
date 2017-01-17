-- main.lua --

dofile('mqtt_config.lua')
dofile('UART_decode.lua')
-- dofile('URL_decode.lua') --old
-- dofile('html_codes.lua') --old


-- Inicia http server
srv=nil
collectgarbage()
srv=net.createServer(net.TCP)
srv:listen(80,function(conn)
	conn:on("receive", function(client,request)
        ------------------------
		--  Códigos HTML aqui --
        ------------------------
	end)
        ------------------
	conn:on("sent",function(conn) conn:close() end)
end)
---------------------------------------------------

