function Decoder(bytes, port) {
    var latitude_int = (bytes[3] << 24) | (bytes[4] << 16) | (bytes[5] << 8) | (bytes[6]);
    var latitude = latitude_int / 100000.0;
    var longitude_int = (bytes[7] << 24) | (bytes[8] << 16) | (bytes[9] << 8) | (bytes[10]);
    var longitude = longitude_int / 100000.0;
    var decoded =  {};
        decoded.battery =(bytes[1]*4.8/100.0);
        decoded.status = bytes[2]==0?"Normal":"Crash Detected";
        decoded.longitude = longitude;
        decoded.latitude = latitude;
        decoded.time = "";
    return decoded;
  }
