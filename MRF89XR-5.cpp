#include "mbed.h"
#include "ConstantDefin.cpp"




class MRF89XR: public SPI
{
    public:
        
        //Default initializer. Takes input for MOSI, MISO, SCLK, CSCONFIG, CSDATA, sets format and frequency
        MRF89XR(PinName mosi = p11, PinName miso = p12, PinName sclk = p13, PinName csConfig = p14, PinName csData = p15): SPI(mosi, miso, sclk)
        {
            Serial pc(USBTX,USBRX);
            pc.printf("test1\n");
            
            ConfigDO = new DigitalOut(csConfig);
            DataDO = new DigitalOut(csData);
            
            pc.printf("test2\n");
            
            ConfigMode();
            
            pc.printf("test3\n");
            
            format(8,0);
            frequency(1000000);
        }
        
        
        /*********************************************************************
         * WORD RegisterRead()
         *
         * Overview:        
         *              This function access the control register of MRF89XA.
         *              The register address and the register settings are
         *              the input
         *
         * PreCondition:    
         *              None
         *
         * Input:       
         *          BYTE    reg     The address of the register 
         *
         * Output:  
         *          BYTE    Output of the Register       
         *
         * Side Effects:    Register settings have been modified
         *
         ********************************************************************/
        char RegisterRead(char reg)
        {
            //first set cs conn low (assuming it's already high)
            CSConfigLow();
            //send configured address bit for the register
            reg = 0x40 | reg << 1; // telling its a read
            write(reg);
            //read the data back by sending dummy byte to 0x00
            char ans = write(0x00);
            //set cs high
            CSConfigHigh();
            return ans;
        }
        
        
        /*********************************************************************
         * void RegisterWrite(BYTE reg, BYTE data)
         *
         * Overview:        
         *              This function access the control register of MRF89XA.
         *              The register address and the register settings are
         *              the input
         *
         * PreCondition:    
         *               None
         *
         * Input:       
         *          BYTE    reg         The address of the register 
         *
         *          BYTE    data        The setting of the register 
         *
         * Output:  None    
         *
         * Side Effects:    Register settings have been modified
         *
         ********************************************************************/
        void RegisterWrite(char reg, char data, int fieldWidth = 2, int LSB = 1 )
        {
            Serial pc(USBTX, USBRX);
            
            /*int currentValue = RegisterRead(reg);
            int protectionMask = ~((~((0xFF << fieldWidth) & 0xFF)) << LSB);
            int protectedValue = (currentValue & protectionMask);
            int newValue = (protectedValue | (data << LSB));*/
            
            
            CSConfigLow();
            write(0xBF & reg << 1);
            
            //pc.printf("REGISTER = 0x%02x NewValue = 0x%04x\n", reg, newValue );
            
            write(data);
            CSConfigHigh();
        }
        
        
        /*********************************************************************
         * void WriteFIFO(BYTE Data)
         *
         * Overview:        
         *              This function fills the FIFO
         *
         * PreCondition:    
         *              MRF89XA transceiver has been properly initialized
         *
         * Input:       
         *              BYTE   Data - Data to be sent to FIFO.
         *
         * Output:      None
         *
         * Side Effects:    
         *              The packet has been sent out
         *
         ********************************************************************/
        void WriteFIFO(char data)
        {
            CSDataLow();
            write(data);
            CSDataHigh();
        }


        /*********************************************************************
         * BYTE ReadFIFO(void)
         *
         * Overview:        
         *              This function reads the Reiceved frame from the fifo 
         *
         * PreCondition:    
         *              MRF89XA transceiver has been properly initialized
         *
         * Input:       
         *              None
         *
         * Output:      BYTE    Data from FIFO
         *
         * Side Effects:    
         *              The packet has been sent out
         *
         ********************************************************************/
        char ReadFIFO(void)
        {
            char value;
            CSDataLow();
            value = write(0x00);
            CSDataHigh();
            return value;
        }
        
        
        /*********************************************************************
         * void SetRFMode(BYTE mode)
         *
         * Overview:        
         *              This functions sets the MRF89XA transceiver operating mode to sleep, transmit, receive or standby
         *
         * PreCondition:    None
         *
         * Input:           None
         *
         * Output:          None
         *
         * Side Effects:    None
         *
         ********************************************************************/
        void SetRFMode( char mode)
        {
            char mcparam_read;
            mcparam_read = RegisterRead(REG_MCPARAM0);
            
            switch(mode)
            {
                case RF_TRANSMITTER:
                    RegisterWrite(REG_MCPARAM0, (mcparam_read & 0x1F) | RF_TRANSMITTER);
                    RF_Mode = RF_TRANSMITTER;               //RF in TX mode
                    break;
                case RF_RECEIVER:
                    RegisterWrite(REG_MCPARAM0, (mcparam_read & 0x1F) | RF_RECEIVER);
                    RF_Mode = RF_RECEIVER;                  //RF in RX mode
                    break;
                case RF_SYNTHESIZER:
                    RegisterWrite(REG_MCPARAM0, (mcparam_read & 0x1F) | RF_SYNTHESIZER);
                    RF_Mode = RF_SYNTHESIZER;               //RF in Synthesizer mode
                    break;
                case RF_STANDBY:
                    RegisterWrite(REG_MCPARAM0, (mcparam_read & 0x1F) | RF_STANDBY);
                    RF_Mode = RF_STANDBY;                   //RF in standby mode
                    break;
                case RF_SLEEP:
                    RegisterWrite(REG_MCPARAM0, (mcparam_read & 0x1F) | RF_SLEEP);
                    RF_Mode = RF_SLEEP;                     //RF in sleep mode
                    break;
            }
        }
        
        
        /*void Send_Packet(char txPacketLength)
        {
            //new char txPacket[txPacketLength];
            SetRFMode(RF_STANDBY);
            RegisterWrite(REG_PKTPARAM3, ((InitConfigRegister[REG_PKTPARAM3] & 0xBF)| FIFO_STBY_ACCESS_WRITE));
            RegisterWrite(REG_IRQPARAM0, (InitConfigRegister[REG_IRQPARAM0] | IRQ1_FIFO_OVERRUN_CLEAR ));
            RegisterWrite(REG_IRQPARAM1, ((InitConfigRegister[REG_IRQPARAM1]) | 0x02));
            WriteFIFO(txPacketLength + 1);
            WriteFIFO(0x00); //Node Address
            
            for(int i=0; i< txPacketLength; i++)
            {
                WriteFIFO(txPacket[i]);
            }
            
            if(Enable_DutyCycle)
            {
                //Initialize timer 
                
            }
        }
        */
 
    private:
        
        PinName csData;
        PinName csConfig;
        DigitalOut* ConfigDO;
        DigitalOut* DataDO;
        
        char RF_Mode;
        
        bool Enable_DutyCycle;
        
        /*
        might want to dynamically allocate these arrays at runtime for access by other functions
        int PACKET_LEN;
        new char TxPacket[PACKET_LEN];
        new char RxPacket[PACKET_LEN];
        */
        
        //modes
        void ConfigMode()
        {
            ConfigDO->write(1);
            DataDO->write(0);
        }
        void DataMode()
        {
            ConfigDO->write(0);
            DataDO->write(1);
        }
        
        //safe methods for setting the pins high or low
        void CSConfigHigh() {ConfigDO->write(1);}
        void CSConfigLow() {ConfigDO->write(0);}
        void CSDataHigh() {DataDO->write(1);}
        void CSDataLow() {DataDO->write(0);}
        
        char InitConfigRegister[31]; 


        void SetInitConfigReg()
        {
           char InitConfigRegister[] = {
        /* 0 */             CHIPMODE_STBYMODE | FREQBAND_915 | VCO_TRIM_11, 
        /* 1 */             MODSEL_FSK | DATAMODE_PACKET | IFGAIN_0,
        /* 2 */             FREGDEV_80,
        /* 3 */             BITRATE_25,
        /* 4 */             OOKFLOORTHRESH_VALUE,
        /* 5 */             FIFOSIZE_64 | FIFO_THRSHOLD_1,
        /* 6 */             0,
        /* 7 */             0,
        /* 8 */             0,
        /* 9 */             0,
        /* 10 */            0,
        /* 11 */            0,
        /* 12 */            DEF_PARAMP | PA_RAMP_23,
        /* 13 */            IRQ0_RX_STDBY_SYNCADRS | IRQ1_RX_STDBY_CRCOK | IRQ1_TX_TXDONE,                     
        /* 14 */            DEF_IRQPARAM1 | IRQ0_TX_START_FIFOTHRESH | IRQ1_PLL_LOCK_PIN_ON,
        /* 15 */            RSSIIRQTHRESH_VALUE,
        /* 16 */            PASSIVEFILT_378 | RXFC_FOPLUS100,
        /* 17 */            DEF_RXPARAM1 | FO_100,
        /* 18 */            DEF_RXPARAM2 | POLYPFILT_OFF | SYNC_SIZE_32 | SYNC_ON | SYNC_ERRORS_0,
        /* 19 */            DEF_RXPARAM3,
        /* 20 */            0,
        /* 21 */            OOK_THRESH_DECSTEP_000 | OOK_THRESH_DECPERIOD_000 | OOK_THRESH_AVERAGING_00,
        /* 22 */            0x69, // 1st byte of Sync word,
        /* 23 */            0x81, // 2nd byte of Sync word,
        /* 24 */            0x7E, // 3rd byte of Sync word,
        /* 25 */            0x96, // 4th byte of Sync word,
        /* 26 */            FC_400 | TXPOWER_13,
        /* 27 */            CLKOUT_ON | CLKOUT_12800,
        /* 28 */            MANCHESTER_OFF | 64,
        /* 29 */            NODEADRS_VALUE,
        /* 30 */            PKT_FORMAT_VARIABLE | PREAMBLE_SIZE_4 | WHITENING_OFF | CRC_ON | ADRSFILT_NONE,
        /* 31 */            FIFO_AUTOCLR_ON | FIFO_STBY_ACCESS_WRITE 
                            };  
        }
        

                             
        
 
 
 };   