# sun_master_slave

- sun_slave: contiene l'implementazione degli slave Meca500 e ATINano43.

- sun_ethercat_master: contiene l'implementazione del master EtherCAT.

- sun_controller: contiene l'implementazione del controllore

            1. Controller.cpp -> Controllore: puro guadagno
                                 Riferimento: gradino smooth
                                 TESTATO e FUNZIONANTE
                                 
            2. Controller_force.cpp -> NON TESTATO
            
            3. Controller_posizione_polinomio.txt -> come Controller.cpp (VERSIONE DI PROVA) 
            
            4. Controller_posizione_seno.txt -> Controllore: puro guadagno
                                                Riferimento: sinusoidale
                                                NON TESTATO
                      
- sun_EtherCAT_Gateway: contiene varie implementazioni per l'inserimento dell'ATINano43 nella rete EtherCAT tramite EasyCAT.
            
            1. sun_EtherCAT_Gateway_noPrint -> codice senza stampe per valutare tempi (TESTATO e FUNZIONANTE)
                                 
            2. sun_EtherCAT_Gateway_flag -> versione alternativa per la comunicazione tramite flag (NON TESTATO)
            
            3. sun_EtherCAT_Gateway_versione1 -> NON TESTATO 
            
- src: contiene i main
           
           1. main_force_control -> regolatore di forza (NON TESTATO)
                                 
            2. main_k_estimation -> stima della costante elastica k per simulare il contatto tra Meca500 e sensore di forza (NON TESTATO)
            
            3. main_master -> movimento giunti (TESTATO E FUNZIONANTE)
            
            4. main_position_control -> controllo in posizione (TESTATO E FUNZIONANTE)
            
            5. Ulteriori file .txt -> VERSIONI DI PROVA NON TESTATE
