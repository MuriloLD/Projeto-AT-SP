-- main.lua --

--Pagina Web
teste = "XXX"
htmlCode = [[<!DOCTYPE html>
    <html>
    <head>
    <style>
    header {
        background-color:black;
        color:white;
        text-align:center;
        padding:5px;     
    }
    nav {
        line-height:30px;
        background-color:#eeeeee;
        height:300px;
        width:100px;
        float:left;
        padding:15px;          
    }
    nav a {
    background-color: black;
    color: white;
    padding: 10px 20px;
    text-decoration: none;
    border-radius: 4px 4px 0 0;
    }
    nav a:hover{
        background-color: orange;
    }
    section {
        width:350px;
        float:left; 
        padding:10px;        
    }
    footer {
        background-color:black;
        color:white;
        clear:both;
        text-align:center;
        padding:5px;         
    }
    </style>
    </head>
    <body>

    <header>
    <h1>Projeto LadyBug: <br> WebService ESP8266</h1>
    </header>

    <nav>
    <a href="/teste">Menu1</a><br>
    <a href="/teste2">Menu2</a><br>
    <a href="/teste3">Menu3</a><br>
    </nav>

    <section>
    <h1>Teste</h1>
    <p>Testando ... ]]..teste..[[</p>
    </section>

    <footer>
    <footer>
    Murilo Leonardelli Daltio<br>
    UFES - 2015
    </footer>

    </body>
    </html>]]
--------------------------------------

-- Verifica UART periodicamente:

-------------------------------

-- Inicia http server
    -- if srv == nil then
    -- 	srv=net.createServer(net.TCP)
    -- else
    	srv:listen(80,function(conn)
    	  conn:on("receive",function(conn,payload)
    	    print("payload:\n")
    	    print(payload)
    	    -- conn:send("<h1> Hello, NodeMCU!!! </h1>")
    	    conn:send(htmlCode)
    	  end)
    	  conn:on("sent",function(conn) conn:close() end)
    	end)
    -- end
---------------------------------------------------

