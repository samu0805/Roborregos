
// ZONA B !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 0 detectar negro, 1 detectar color
//cambiar color en el caso de que sea diferente número para blanco y negro
// 5 igual a blanco, 4 a negro
void path(){
    int infra1=digitalRead(infrared1);
    int infra2=digitalRead(infrared2);
    int col = getColor();
    Serial.print(infra2);
    Serial.print(",");
    Serial.print(col);
    Serial.print(",");
    Serial.println(infra1);
     //básico
    if (infra1 == 0 && color == 4 && infra2 == 0){
        ahead();
    }//avanza V
    else if(infra1 == 1 && color != 4 && infra2 == 1){
        bool found = false;
        setleft();
        while (!found){ //sacar un rango de pasos para llegar al final
            setleft();
            infra1 = getLinea(infra1);
            color = getColor(color);
            if(infra1 == 0 && color == 4){
                break;
            }
        }
    }// Línea en la derecha
    else if(infra1 == 0 && color == 4 && infra2 == 1){
        while(infra2 == 1){
            setleft();
            infra2 = getLinea(infra2);
        }
    }// Línea en la izquierda PID
    else if(infra1 == 1 && color == 4 && infra2 == 0){
        while(infra1 == 1){
            setright();
            infra1 = getLinea(infra1);
        }
    } // Línea Punteada, Blanco los tres
    else if(infra1 == 0 && color != 4 && infra2 == 0){
        int count = 0;
        bool linea = false; 
        while (count < 100 && linea == false){
            setahead();
            col = getcolor();
            if(col == 4){
                linea = true; 
            }
            count++; 
        }
    }// peor de los casos
    else if (infra1 == 1 && color == 4 && infra2 == 1){
        // tipos de patrones
        // tipo 1: linea perpendicular con línea consecuente en el frente.
        // tipo 2: circulo o cuadrado con el centro despejado sin línea, recorrer el contorno
        setahead();
    }// Error, 001 and 100 
    else if (infra1 == 0 && color != 4 && infra2 == 1){
        setright();
    }
    else if (infra1 == 1 && color != 4 && infra2 == 0){
        setleft();
    }
    else{
        cout << "error";
    }
}

void zonaB(){
    SPEED_MT = 90
    SPEED_MT2 = SPEED_MT
    set_speed();
    while(paredAdelante() == false){
        path();
    }
}
