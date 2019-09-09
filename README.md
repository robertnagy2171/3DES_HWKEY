# 3DES_HWKEY
Microchip PIC 16F84/12F809a based hardware key for triple DES encryption/decryption.
It is connected to the RS232 serial port of the PC.

Commands:
--

**I - initialisation, warm reset**  
  
_usage (< input, > output):_  
`< I\r`<br>
`> \n`<br>
`> OK\r\n`<br>

**V - version info**  
Sends the sw version back.  
  
_usage (< input, > output):_  
`< V\r`<br>
`> \n`<br>
`> 0.9D\r\n`<br>
`> OK\r\n`<br>

**M&lt;x&gt; - set mode to &lt;x&gt;**  
Sets the mode of encryption/decryption where &lt;x&gt; means:  
  '0' - DES, binary  
  '1' - DES, ASCII  
  '2' - 3DES, binary  
  '3' - 3DES, ASCII  
If &lt;x&gt; is not provided then it returns the current value of mode.  
  
_usage (< input, > output):_  
`< M1\r`<br>
`> \n`<br>
`> OK\r\n`<br>
`< M\r`<br>
`> \n`<br>
`> 1\r\n`<br>
`> OK\r\n`<br>
  
**E&lt;k&gt;&lt;data&gt; - DES encryption**  
&lt;k&gt; is the ID of the key ('0' or '1') for encryption.  
The 56-bit keys are stored on 7 bytes in the program memory.  
The sequence of triple DES encryption is the following:  
  - encryption with key &lt;k&gt;  
  - decryption with key other than &lt;k&gt;  
  - encryption with key &lt;k&gt;  
  
&lt;data&gt; is the 64-bit data to be encrypted.  
It is represented on 8 bytes in binary mode and 11 bytes (ascii chars) in ASCII mode.  
In ASCII mode the 64 bits are split according to the following schema: 6 6 6 6 6 6 6 6 6 6 4. Tha last 4 bits shifted left 2 times (right padded with 0).  
All 6-bit values are incremented with 32 for moving the value into the 32-95 (20h - 5Fh) interval.  
  
_usage (< input, > output):_  
`< E<k><data>\r`<br>
`> \n`<br>
`> <encrypted data>\r\n`<br>
`> OK\r\n`<br>
  
**D&lt;k&gt;&lt;encrypted data&gt; - DES decryption**  
&lt;k&gt; is the ID of the key ('0' or '1') for decryption.  
The 56-bit keys are stored on 7 bytes in the program memory.  
The sequence of triple DES decryption is the following:  
  - decryption with key &lt;k&gt;  
  - encryption with key other than &lt;k&gt;  
  - decryption with key &lt;k&gt;  
  
&lt;encrypted data&gt; is the 64-bit data to be decrypted.  
It is represented on 8 bytes in binary mode and 11 bytes (ascii chars) in ASCII mode.  
In ASCII mode the 64 bits are split according to the following schema: 6 6 6 6 6 6 6 6 6 6 4. Tha last 4 bits shifted left 2 times (right padded with 0).  
All 6-bit values are incremented with 32 for moving the value into the 32-95 (20h - 5Fh) interval.  
  
_usage (< input, > output):_  
`< D<k><decrypted data>\r`<br>
`> \n`<br>
`> <data>\r\n`<br>
`> OK\r\n`<br>
