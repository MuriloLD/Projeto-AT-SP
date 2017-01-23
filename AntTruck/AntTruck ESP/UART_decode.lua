-----------------------------------------------------------------
------  Script used to decode incoming serial data  -------------
------	   Double/Float numbers not supported yet   -------------
-----------------------------------------------------------------

--[==[ ATENTION
	Changing numbers of variables decoded:
		1. Add meaning (name) of each one
		2. Add Matching pattern
		3. Change numbers os bytes
]==]


-------------------------------------------------------------
	--Buffer:
	dataBuffer = '???' --GLOBAL 
-------------------------------------------------------------


--Debug Tables:
function printTable(table)
	for i,v in pairs(table) do
		print(i,v)
	end
	print('END\n')
end
------------------------------------------------------------------
--MQTT Publish Table:
function publishTable(table)
	for key,value in pairs(table) do
		mqttClient:publish('AntTruck/Vars/'..key,value,0,0)
	end
	collectgarbage()
	collectgarbage()
end
------------------------------------------------------------------
-- Convert to signed number when needed:
function convertToSigned(var,size) -->Float/Double not supported
	local signBit = size*8
	if bit.isset(var,signBit-1) then
		if size == 4 then
			var = 0 - (bit.bxor(var, 0xffffffff) + 1)
		elseif size == 2 then
			var = 0 - (bit.bxor(var, 0xffff) + 1)
		elseif size == 1 then
			var = 0 - (bit.bxor(var, 0xff) + 1)
		end
	end
	return var
end
------------------------------------------------------------------


-- Interpret incoming bytes:
function serial_Decode(data)
	------------------
	local decoded={} -- saves unnamed vars received
	local variables = {} -- saves named vars decoded
	------------------
	if data=='quit\n' then
		print('Uart back to normal!')
		uart.on("data",_,_,1)
	else
		--Insert every byte from string in a Table:
		for i=1,string.len(data) do
			decoded[i]=string.sub(data,i,i)
		end

		if decoded[1] == '[' then
			--Save numbers of vars and its size:
			decoded['nvars'] = string.find(data,']')-2 --> -2 positions for the '[',']'
			for i=1,decoded.nvars do
			decoded['n'..i] = tonumber(decoded[i+1]) --> Skip 'decoded[1]=='['
			end
			--printTable(decoded) --> Debug

			--Start decoding variables according to each size:
			local bufferCount = decoded.nvars+3 --> +3: skip '[',']' and set into the first byte position
			local varsCount=1

			while varsCount<=decoded.nvars do ----> Main loop that reconstruct the variables
				local mask = 0
				for i=1,decoded['n'..varsCount]-1 do --> -1: last position doesn't have to be shifted
					mask = bit.bor(mask,string.byte(decoded[bufferCount]))
					mask = bit.lshift(mask,8)
					bufferCount = bufferCount+1 --> update decoded counter
				end
				mask = bit.bor(mask,string.byte(decoded[bufferCount])) --> last byte
				bufferCount = bufferCount+1 --> update decoded counter

				-- Var Decoded. Now save it as 'var..index':
				decoded['var'..varsCount] = mask
				varsCount=varsCount+1 --> next variable
			end
		else
			-- print('Invalid sequence of data: '..data)
			print('Invalid sequence of data!')
		end
	end
	-----------------------------------------------------------------------------
	--Meaning for each of the decoded variables
	if decoded.var1~=nil then
	variables.nav_pos_x = convertToSigned(decoded.var1, 4)
	end
	if decoded.var2~=nil then
	variables.nav_pos_y = convertToSigned(decoded.var2, 4)
	end
	if decoded.var3~=nil then
	variables.nav_velLinear = convertToSigned(decoded.var3, 4)
	end
	if decoded.var4~=nil then
	variables.nav_velAngular = convertToSigned(decoded.var4, 4)
	end
	if decoded.var5~=nil then
	variables.rPID_Setpoint = convertToSigned(decoded.var5, 4)
	end
	if decoded.var6~=nil then
	variables.lPID_Setpoint = convertToSigned(decoded.var6, 4)
	end
	if decoded.var7~=nil then
	variables.rPID_Input = convertToSigned(decoded.var7, 4)
	end
	if decoded.var8~=nil then
	variables.lPID_Input = convertToSigned(decoded.var8, 4)
	end
	if decoded.var9~=nil then
	variables.rPID_Output = convertToSigned(decoded.var9, 4)
	end
	if decoded.var10~=nil then
	variables.lPID_Output = convertToSigned(decoded.var10, 4)
	end
	if decoded.var11~=nil then
	variables.dist_US_Frente = string.format('%u',decoded.var11)
	end
	if decoded.var12~=nil then
	variables.dist_US_Direito = string.format('%u',decoded.var12)
	end
	if decoded.var13~=nil then
	variables.dist_US_Esquerdo = string.format('%u',decoded.var13)
	end
	if decoded.var14~=nil then
	variables.dist_US_servo = string.format('%u',decoded.var14)
	end
	if decoded.var15~=nil then
	variables.vBateria = convertToSigned(decoded.var15, 4)
	end
	if decoded.var16~=nil then
	variables.nav_heading = convertToSigned(decoded.var16, 4)
	end
	----------------------------------------------------------
	local _, err = pcall( function() publishTable(variables) end)
	if err then print('Error: '..err) end
	--printTable(variables) --> Debug
end
-------------------------------------------------------------

-------------------------------------------------------------
function p_serial_Decode(data)

	dataBuffer = dataBuffer .. data
	encodedData = string.match(dataBuffer,
		'>>(..........................................................................)!!')

	if encodedData ~= nil then
		print("ACHOU!: ".. encodedData) -- DEBUG
		-------
		local _, err = pcall(serial_Decode,encodedData)
		if err then
			mqttClient:publish('AntTruck/talk',err,0,0)
		end
	else
		print('NAO ACHOU: '..dataBuffer..'  size: '..string.len(dataBuffer)) -- DEBUG
	end

	--Clean Buffer:
	if string.len(dataBuffer) > 64*3 then
		dataBuffer = '???'
	end

	collectgarbage()
	collectgarbage()
end

-- Serial handler function
uart.on("data", 78, p_serial_Decode, 0)
-------------------------------------------------------------