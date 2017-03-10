#include "MRF89XR.cpp"
 

MRF89XR sensor(p11,p12,p13,p14,p15);
Serial pc(USBTX,USBRX);

int main() 
{
    
    
    
    
    
    
    
    
    /*
    send commands read(register) or write(register,value) to mbed module.
    This is simple, so your register and values must both be exactly 3 digits
    in decimal form. ex: if you want to write to register 0x0A, value 0x0B, 
    then it should look like the following:
    
    write(010,011)
    
    */
    /*
    pc.printf("hello world\n");
    // Send 0x8f, the command to read the WHOAMI register
    //int whoami = spi.write(0x8F);

    char c[128];
    int i = 0;
    int restart = 0;
    
    

    while(1)
    {
        
        c[i] = pc.getc();
        pc.putc(c[i]);
        
        if(c[i]==13)
        { //if you press the enter button
            restart = 1;
            pc.printf("\n");
            if (c[0] == 'r' && c[1] == 'e'&& c[2] == 'a'&& c[3] == 'd')
            {
                int regNum = int(c[7]) -48  + 10 * (int(c[6])-48) + 100 * (int(c[5])-48);
                //pc.printf("regNum = 100 * %i, 10* %i, %i", c[5]-48,c[6]-48,c[7]-48);
                pc.printf("reading 0x%02x = 0x%02x \n", regNum, sensor.RegisterRead(regNum));
            }
            else if (c[0] == 'w' && c[1] == 'r'&& c[2] == 'i' && c[3] == 't'&& c[4] == 'e')
            {
                int regNum = int(c[8])- 48 + 10* (int(c[7])-48) + 100 * (int(c[6])-48);
                int writeNum = int(c[12])-48 + 10* (int(c[11])-48) + 100 * (int(c[10])-48);
                pc.printf("writing to 0x%02x and value 0x%02x \n", regNum, writeNum );
                sensor.RegisterWrite(regNum,writeNum);
                
            }
        }
        
        if (restart) 
        {
            i = 0;
            restart = 0;
        }
        else
            i++;
            
    }
*/
    
}