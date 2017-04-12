-- SupetTruck WebSocket config

--Create client
ws = websocket.createClient()
ws:config({headers={['User-Agent']='SupetTruck'}})


-- Callbacks:
ws:on("connection", function(ws)
  print('got ws connection')
end)
ws:on("receive", function(_, msg, opcode)
  print('got message:', msg, opcode) -- opcode is 1 for text message, 2 for binary
end)
ws:on("close", function(_, status)
  print('connection closed', status)
  ws = nil -- required to lua gc the websocket client
end)


--Connect to WS Server
ws:connect('ws://'..wsip)
