-- HTML SOURCE CODES
--Paginas Web

-----------------------------------------------
--             HTML VELOCIDADE               --
-----------------------------------------------

  setpointDireito = "x"
  setpointEsquerdo = "x"

function html_Velocidade()
  local htmlCode=[[
  <!DOCTYPE html>
<html>
<head>
<meta http-equiv="refresh" content=5;>
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
  width:120px;
  float:left;
  padding:20px;
}
nav a {
  background-color: black;
  color: white;
  float: center;
  text-align: center;
  padding: 5px 20px;
  text-decoration: none;
  border-radius: 4px 4px;
}
nav a:hover{
  background-color: orange;
}
section {
  width:350px;
  float:left; 
  padding:20px 10px 100px 20px;        
}
article {
  padding:20px 10px 100px 10px;
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
  <h1>Projeto SuperTruck: <br> WebService ESP8266</h1>
</header>

<nav>
  <h2>&nbsp; Menu</h2>
  <a href="/odometria">Odometria&nbsp;</a><br>
  <a href="/velocidade">Velocidade</a><br>
  <a href="/ultrassom">Ultrassom&nbsp;&nbsp;</a><br>
</nav>

<section>
  <h1>Setpoints Atuais</h1>
  <p>Setpoint Motor Direito: ]]..setpointDireito..[[</p>
  <p>Setpoint Motor Esquerdo: ]]..setpointEsquerdo..[[</p>
</section>

<article>
  <h1>Inserir Velocidades Desejadas:</h1>
  <form>
  Setpoint Motor Direito:<br>
  <input type="text" name="setpointDireito">
  <br>
  <br>
  Setpoint Motor Esquerdo:<br>
  <input type="text" name="setpointEsquerdo">
  <br><br>
  <input type="submit">
  </form>
</article>

<footer>
  Murilo Leonardelli Daltio<br>
  UFES - 2016
</footer>

</body>
</html>]]
   
   return htmlCode
end
--------------------------------------

-----------------------------------------------
--             HTML ODOMETRIA                --
-----------------------------------------------

  counts_Dir = -99
  counts_Esq = -99

function html_Odometria()
  local htmlCode=[[
  <!DOCTYPE html>
<html>
<head>
<meta http-equiv="refresh" content=5;>
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
  width:120px;
  float:left;
  padding:20px;
}
nav a {
  background-color: black;
  color: white;
  float: center;
  text-align: center;
  padding: 5px 20px;
  text-decoration: none;
  border-radius: 4px 4px;
}
nav a:hover{
  background-color: orange;
}
section {
  width:350px;
  float:left; 
  padding:20px 10px 100px 20px;        
}
article {
  padding:20px 10px 100px 10px;
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
  <h1>Projeto SuperTruck: <br> WebService ESP8266</h1>
</header>

<nav>
  <h2>&nbsp; Menu</h2>
  <a href="/odometria">Odometria&nbsp;</a><br>
  <a href="/velocidade">Velocidade</a><br>
  <a href="/ultrassom">Ultrassom&nbsp;&nbsp;</a><br>
</nav>

<section>
  <h1>Odometria</h1>
</section>


<footer>
  Murilo Leonardelli Daltio<br>
  UFES - 2016
</footer>

</body>
</html>]]

return htmlCode
end

-----------------------------------------------
--             HTML ULTRASSOM                --
-----------------------------------------------

 US_Frente = '--'
 US_Direito = '--'
 US_Esquerdo = '--'
 US_Tras = '--'

function html_Ultrassom()
  local htmlCode=[[
  <!DOCTYPE html>
<html>
<head>
<meta http-equiv="refresh" content=5;>
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
  width:120px;
  float:left;
  padding:20px;
}
nav a {
  background-color: black;
  color: white;
  float: center;
  text-align: center;
  padding: 5px 20px;
  text-decoration: none;
  border-radius: 4px 4px;
}
nav a:hover{
  background-color: orange;
}
section {
  width:350px;
  float:left; 
  padding:20px 10px 100px 20px;        
}
article {
  padding:20px 10px 100px 10px;
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
  <h1>Projeto SuperTruck: <br> WebService ESP8266</h1>
</header>

<nav>
  <h2>&nbsp; Menu</h2>
  <a href="/odometria">Odometria&nbsp;</a><br>
  <a href="/velocidade">Velocidade</a><br>
  <a href="/ultrassom">Ultrassom&nbsp;&nbsp;</a><br>
</nav>

<section>
  <h1>Ultrassom</h1>
</section>


<footer>
  Murilo Leonardelli Daltio<br>
  UFES - 2016
</footer>

</body>
</html>]]
	
    return htmlCode
end

-----------------------------------------------
--                HTML HOME                  --
-----------------------------------------------

function html_Home()
  local htmlCode=[[
  <!DOCTYPE html>
<html>
<head>
<meta http-equiv="refresh" content=5;>
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
  width:120px;
  float:left;
  padding:20px;
}
nav a {
  background-color: black;
  color: white;
  float: center;
  text-align: center;
  padding: 5px 20px;
  text-decoration: none;
  border-radius: 4px 4px;
}
nav a:hover{
  background-color: orange;
}
section {
  width:350px;
  float:left; 
  padding:20px 10px 100px 20px;        
}
article {
  padding:20px 10px 100px 10px;
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
  <h1>Projeto SuperTruck: <br> WebService ESP8266</h1>
</header>

<nav>
  <h2>&nbsp; Menu</h2>
  <a href="/odometria">Odometria&nbsp;</a><br>
  <a href="/velocidade">Velocidade</a><br>
  <a href="/ultrassom">Ultrassom&nbsp;&nbsp;</a><br>
</nav>

<section>
  <h1>Home</h1>
</section>


<footer>
  Murilo Leonardelli Daltio<br>
  UFES - 2016
</footer>

</body>
</html>]]
   
   return htmlCode
end