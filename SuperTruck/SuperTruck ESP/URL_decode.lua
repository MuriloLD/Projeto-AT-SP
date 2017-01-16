-- Funcções de Decode

 -- UART decode:
    -- setpointDireito
    function cmd_setpointDir(serialData)
      if serialData ~= nil then
	setpointDireito=serialData
        print("cmd_setpointDir("..setpointDireito..") >> ok!\n")
      else
        print("cmd_setpointDir() >> ERROR: 'nil'!\n")
      end
    end

    -- setpointEsquerdo:
    function cmd_setpointEsq(serialData)
      if serialData ~= nil then
        setpointEsquerdo=serialData
        print("cmd_setpointEsq("..setpointEsquerdo..") >> ok!\n")
      else
        print("cmd_setpointEsq() >> ERROR: 'nil'!\n")
      end
    end
-------------------------------

--função URL decode:
function urlDecode(payload, str) 
  local i,j = string.find(payload, str)
  if j ~= nil then
    local k=string.find(payload,"&",i)
      if k ~= nil then
	return string.sub(payload,j+1,k-1)
      end
  else
    return nil
  end
end
-----------------------

-- Requisição de página
function currentPage(payload, str) 
  local i,j = string.find(payload, str)
  if i ~= nil then
    print('Current page: '..str..' ok!')
    return true
  else
    return nil
  end
end
-----------------------

--------------------------------------------------------------------------
--			Line by line functions				--
--------------------------------------------------------------------------
  -- Concat the contents of the parameter list,
-- separated by the string delimiter (just like in perl)
-- example: strjoin(", ", {"Anna", "Bob", "Charlie", "Dolores"})
--[=[
function strjoin(delimiter, list)
  local len = getn(list)
  if len == 0 then 
    return "" 
  end
  local string = list[1]
  for i = 2, len do 
    string = string .. delimiter .. list[i] 
  end
  return string
end
--]=]

-- Divide uma String em partes por um delimitador (pattern)
function str_Split(delimiter, text)
  local list = {}
  local pos = 1
  if string.find("", delimiter, 1) then -- this would result in endless loops
    error("Delimiter matches empty string!")
  end
  while 1 do
    local first, last = string.find(text, delimiter, pos)
    if first then -- found?
      table.insert(list, string.sub(text, pos, first-1))
      pos = last+1
    else
      table.insert(list, string.sub(text, pos))
      break
    end
  end
  return list
end

-- Manda para net.socket
  function htmlSend(htmlCode,netsocket)
    local list = str_Split("\n", htmlCode)
    local i,v
    for i,v in pairs(list) do
      netsocket:send(v..'\n')
    end
  end

--***********************************************************************************
