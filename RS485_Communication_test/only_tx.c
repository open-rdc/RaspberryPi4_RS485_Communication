char data[7] = {};

    data[0] = 0x07;
    data[1] = 0x03;
    data[2] = 0x00;
    data[3] = ID;
    data[4] = addr;
    data[5] = length;
    data[6] = 0;
    for(int i = 0; i < data[0]-1; i++)
        data[6] += data[i];
