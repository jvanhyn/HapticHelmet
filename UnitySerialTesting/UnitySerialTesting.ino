// Read Values from Unity 
float dist = 1000; // Distance from player to next waypoint
int dir = 0;	// Turn direction at waypoint
int unity_heading = 0; // Player in game heading

/*
VARIABLES READ FROM UNITY
*/
// UNITY PARSING VARS
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;


void setup() {
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  delay(5000);
}

void loop() {
  recvWithStartEndMarkers();
     if (newData == true) {
      
       strcpy(tempChars, receivedChars);
           // this temporary copy is necessary to protect the original data
           //   because strtok() used in parseData() replaces the commas with \0
       parseData();
      //  Serial.println("received!");
       // Serial.print("Data Recived - ");
       // showParsedData();
       newData = false;
     }
    //  Serial.println(dist);
     if (unity_heading > 20) {
       digitalWrite(3, HIGH);    
     }
     else {
       digitalWrite(3, LOW); 
     }

    //  digitalWrite(3, HIGH);
    //  delay(200);
    //  digitalWrite(3, LOW);
    //  delay(1000);
    //  if (dir == 0) {
    //    digitalWrite(12, HIGH);   
    //    delay(500);
    //    digitalWrite(12, LOW); 
    //    delay(500);
    //  }
    //  else if (dir == 90) {
    //    digitalWrite(12, HIGH);   
    //    delay(1000);
    //    digitalWrite(12, LOW); 
    //    delay(1000);
    //  }
    //  else if (dir == 180) {
    //    digitalWrite(12, HIGH);   
    //    delay(1500);
    //    digitalWrite(12, LOW); 
    //    delay(1500);
    //  }
    //  else if (dir == 270) {
    //    digitalWrite(12, HIGH);   
    //    delay(2000);
    //    digitalWrite(12, LOW); 
    //    delay(2000);
    //  }
     
             
}

/*
	This Code block deals with grabbing and parsing data from Unity
*/


void recvWithStartEndMarkers() {
   static boolean recvInProgress = false;
   static byte ndx = 0;
   char startMarker = '(';
   char endMarker = ')';
   char rc;


   while (Serial.available() > 0 && newData == false) {
       rc = Serial.read();


       if (recvInProgress == true) {
           if (rc != endMarker) {
               receivedChars[ndx] = rc;
               ndx++;
               if (ndx >= numChars) {
                   ndx = numChars - 1;
               }
           }
           else {
               receivedChars[ndx] = '\0'; // terminate the string
               recvInProgress = false;
               ndx = 0;
               newData = true;
           }
       }


       else if (rc == startMarker) {
           recvInProgress = true;
       }
   }
}


//============


void parseData() {      // split the data into its parts


   char * strtokIndx; // this is used by strtok() as an index


   strtokIndx = strtok(tempChars,",");      // get the first part - the string
   dist = atof(strtokIndx);     // convert this part to a float

   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   dir = atof(strtokIndx);     // convert this part to a float

   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   unity_heading = atof(strtokIndx);     // convert this part to a float


}


// Just prints the data that we parsed from unity
void showParsedData() {
   Serial.print("Dist: ");
   Serial.print(dist);
   Serial.print("  ");
   Serial.print("Dir: ");
   Serial.print(dir);
   Serial.print("  ");
   Serial.print("User Angle: ");
   Serial.print(dir);
}
