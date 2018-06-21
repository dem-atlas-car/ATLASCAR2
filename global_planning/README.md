global_planning 
==================

Criado por Pedro Bouça Nova



Requisitos:
-> Google Maps API key
-> Credenciais de acesso à base de dados local



Subscreve:
/bestpos  	(posição global do veículo)
/destination 	(destino da missão)


Publica:
/waypoints_full			(array de waypoints descodificados)
/waypoints_steps		(array de waypoints que definem as etapas da rota)
/waypoins_previous_next_wsg84	(array de waypoints que definem o segmento de referência)
