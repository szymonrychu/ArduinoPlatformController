

Command command = Command(" ", "\n");

void defaultFunc(char* data){
    Logger::error(data);
}

void G11HandleDistanceAngleTime(){
    float distanceParsed = 0.0f; char* distanceCH = command.next(); distanceParsed = atof(distanceCH);
    char* angleCH = command.next(); angleTarget = atof(angleCH);
    unsigned long timeMS = 0; char* timeMSCH = command.next(); timeMS = atol(timeMSCH);
    distanceTarget += distanceParsed;
    angleVelocityTarget = abs((angleTarget - angleEncoder.getLastPosition()) / ((float)timeMS));       // change miliseconds to microseconds
    distanceVelocityTarget = abs(distanceParsed / ((float)timeMS)); // change miliseconds to microseconds
    sprintf(buffer, "G11:%.5f:%.5f:%.5f:%.5f", 
        distanceTarget, distanceVelocityTarget, angleTarget, angleVelocityTarget);
    Logger::info(buffer);
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