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
		mqttClient:publish('SuperTruck/Vars/'..key,value,0,0)
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
	local buffer={} -- saves unnamed vars received
	local variables = {} -- saves named vars decoded
	------------------
	if data=='quit\n' then
		print('Uart back to normal!')
		uart.on("data",_,_,1)
	else
		--Insert every byte from string in a list:
		for i=1,string.len(data) do
			buffer[i]=string.sub(data,i,i)
		end

		if buffer[1] == '[' then
			--Save numbers of vars and its size:
			buffer['nvars'] = string.find(data,']')-2 --> -2 positions for the '[',']'
			for i=1,buffer.nvars do
			buffer['n'..i] = tonumber(buffer[i+1]) --> Skip 'buffer[1]=='['
			end
			--printTable(buffer) --> Debug

			--Start decoding variables according to each size:
			local bufferCount = buffer.nvars+3 --> +3: skip '[',']' and set into the first byte position
			local varsCount=1

			while varsCount<=buffer.nvars do ----> Main loop that reconstruct the variables
				local mask = 0
				for i=1,buffer['n'..varsCount]-1 do --> -1: last position doesn't have to be shifted
					mask = bit.bor(mask,string.byte(buffer[bufferCount]))
					mask = bit.lshift(mask,8)
					bufferCount = bufferCount+1 --> update buffer counter
				end
				mask = bit.bor(mask,string.byte(buffer[bufferCount])) --> last byte
				bufferCount = bufferCount+1 --> update buffer counter

				-- Var Decoded. Now save it as 'var..index':
				buffer['var'..varsCount] = mask
				varsCount=varsCount+1 --> next variable
			end
		else
			-- print('Invalid sequence of data: '..data)
			print('Invalid sequence of data!')
		end
	end
	-----------------------------------------------------------------------------
	--Meaning for each of the decoded variables
	if buffer.var1~=nil then
	variables.nav_pos_x = convertToSigned(buffer.var1, 4)
	end
	if buffer.var2~=nil then
	variables.nav_pos_y = convertToSigned(buffer.var2, 4)
	end
	if buffer.var3~=nil then
	variables.nav_velLinear = convertToSigned(buffer.var3, 4)
	end
	if buffer.var4~=nil then
	variables.nav_velAngular = convertToSigned(buffer.var4, 4)
	end
	if buffer.var5~=nil then
	variables.rPID_Setpoint = convertToSigned(buffer.var5, 4)
	end
	if buffer.var6~=nil then
	variables.lPID_Setpoint = convertToSigned(buffer.var6, 4)
	end
	if buffer.var7~=nil then
	variables.rPID_Input = convertToSigned(buffer.var7, 4)
	end
	if buffer.var8~=nil then
	variables.lPID_Input = convertToSigned(buffer.var8, 4)
	end
	if buffer.var9~=nil then
	variables.rPID_Output = convertToSigned(buffer.var9, 4)
	end
	if buffer.var10~=nil then
	variables.lPID_Output = convertToSigned(buffer.var10, 4)
	end
	if buffer.var11~=nil then
	variables.dist_US_Frente = string.format('%u',buffer.var11)
	end
	if buffer.var12~=nil then
	variables.dist_US_Direito = string.format('%u',buffer.var12)
	end
	if buffer.var13~=nil then
	variables.dist_US_Esquerdo = string.format('%u',buffer.var13)
	end
	if buffer.var14~=nil then
	variables.dist_US_servo = string.format('%u',buffer.var14)
	end
	if buffer.var15~=nil then
	variables.vBateria = convertToSigned(buffer.var15, 4)
	end
	if buffer.var16~=nil then
	variables.nav_heading = convertToSigned(buffer.var16, 4)
	end
	----------------------------------------------------------
	local _, err = pcall( function() publishTable(variables) end)
	if err then print('Error: '..err) end
	--printTable(variables) --> Debug
end
-------------------------------------------------------------
	--Matching function:
	dataBuffer = '???'

	function matchBuffer()
		matched = string.match(dataBuffer,'>>(..........................................................................)!!')
		-- print(matched)
		return matched
	end


-------------------------------------------------------------
function p_serial_Decode(data)

	dataBuffer = dataBuffer .. data
	encodedData = matchBuffer()

	if encodedData ~= nil then
		print("ACHOU!: ".. encodedData) -- DEBUG
		-------
		local _, err = pcall(serial_Decode,encodedData)
		if err then
			mqttClient:publish('SuperTruck/talk',err,0,0)
		end
	else
		print('NÃO ACHOU: '..dataBuffer..'  size: '..string.len(dataBuffer)) -- DEBUG
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