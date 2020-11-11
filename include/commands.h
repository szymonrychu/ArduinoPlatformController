
/*
G11 1 1 10000

G11 0 0.5235987756 5000
G11 0 0.7853981634 5000
G11 0 1.5707963268 5000
G11 0 -1.5707963268 5000

G11 1 0 10000
G11 2 0 10000
G11 3 0 10000
G11 -3 0 10000
*/

Command command = Command(" ", "\n");

void defaultFunc(char* data){
    Logger::error(data);
}

void G11HandleDistanceAngleTime(){
    double distance = 0.0f; char* distanceCH = command.next(); distance = atof(distanceCH);
    double angle = 0.0f; char* angleCH = command.next(); angle = atof(angleCH);
    unsigned long timeMS = 0; char* timeMSCH = command.next(); timeMS = atol(timeMSCH);
    targetLoops = timeMS/LOOP_DELAY;
    angleVelocity = abs(angle/((float)timeMS/1000.0));
    distanceVelocity = abs(distance/((float)timeMS/1000.0));
    angleDelta = angle/(float)targetLoops;
    distanceDelta = distance/(float)targetLoops;
    Logger::info("G11:RECEIVED_OK;");
}

void G50HandleAnglePIDPrint(){
    PIDvalues data = anglePIDs.getPIDValues();
    sprintf(buffer, "G50:%.5f:%.5f:%.5f", 
        data.kP, data.kI, data.kD);
    Logger::info(buffer);
}

void G51HandleAnglePID(){
    double kP = 0.0f; char* kPCH = command.next(); kP = atof(kPCH);
    double kI = 0.0f; char* kICH = command.next(); kI = atof(kICH);
    double kD = 0.0f; char* kDCH = command.next(); kD = atof(kDCH);
    PIDvalues data = {kP, kI, kD, anglePIDs.getPIDValues().lowLimit, anglePIDs.getPIDValues().highLimit};
    anglePIDs.setPidValues(data);
}

void G52HandleAngleVelocityPIDPrint(){
    PIDvalues data = anglePIDs.getVelocityPIDValues();
    sprintf(buffer, "G52:%.5f:%.5f:%.5f", 
        data.kP, data.kI, data.kD);
    Logger::info(buffer);
}

void G53HandleAngleVelocityPID(){
    double kP = 0.0f; char* kPCH = command.next(); kP = atof(kPCH);
    double kI = 0.0f; char* kICH = command.next(); kI = atof(kICH);
    double kD = 0.0f; char* kDCH = command.next(); kD = atof(kDCH);
    PIDvalues data = {kP, kI, kD, anglePIDs.getVelocityPIDValues().lowLimit, anglePIDs.getVelocityPIDValues().highLimit};
    anglePIDs.setVelocityPidValues(data);
}

void G60HandleDistancePIDPrint(){
    PIDvalues data = distancePIDs.getPIDValues();
    sprintf(buffer, "G60:%.5f:%.5f:%.5f", 
        data.kP, data.kI, data.kD);
    Logger::info(buffer);
}

void G61HandleDistancePID(){
    double kP = 0.0f; char* kPCH = command.next(); kP = atof(kPCH);
    double kI = 0.0f; char* kICH = command.next(); kI = atof(kICH);
    double kD = 0.0f; char* kDCH = command.next(); kD = atof(kDCH);
    PIDvalues data = {kP, kI, kD, distancePIDs.getPIDValues().lowLimit, distancePIDs.getPIDValues().highLimit};
    distancePIDs.setPidValues(data);
}

void G62HandleDistanceVelocityPIDPrint(){
    PIDvalues data = distancePIDs.getVelocityPIDValues();
    sprintf(buffer, "G62:%.5f:%.5f:%.5f", 
        data.kP, data.kI, data.kD);
    Logger::info(buffer);
}

void G63HandleDistanceVelocityPID(){
    double kP = 0.0f; char* kPCH = command.next(); kP = atof(kPCH);
    double kI = 0.0f; char* kICH = command.next(); kI = atof(kICH);
    double kD = 0.0f; char* kDCH = command.next(); kD = atof(kDCH);
    PIDvalues data = {kP, kI, kD, distancePIDs.getVelocityPIDValues().lowLimit, distancePIDs.getVelocityPIDValues().highLimit};
    distancePIDs.setVelocityPidValues(data);
}

void setupCommands(){
    command.addDefaultHandler(defaultFunc);
    command.addCommand("G11", G11HandleDistanceAngleTime);
    command.addCommand("G50", G50HandleAnglePIDPrint);
    command.addCommand("G51", G51HandleAnglePID);
    command.addCommand("G52", G52HandleAngleVelocityPIDPrint);
    command.addCommand("G53", G53HandleAngleVelocityPID);
    command.addCommand("G60", G60HandleDistancePIDPrint);
    command.addCommand("G61", G61HandleDistancePID);
    command.addCommand("G62", G62HandleDistanceVelocityPIDPrint);
    command.addCommand("G63", G63HandleDistanceVelocityPID);

}