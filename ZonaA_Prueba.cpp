//Zona A !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void pelotaEncontrada(){
    right();
    right();
    back();
}
void zonaA(){
    bool foundPelota = false; 
    bool foundSalida = false;
    int countD= 0;
    int countL=0;
    //entrada Laberinto 
    ahead();
    //busquda pelota
    while (foundPelota == false && lineaNegra == false){
        if(paredAdelante() == true){
            right();
            ahead();
            if(lineaNegra == false){
                left();
            }else{
                lineaNegra = true;
                left();
            }
            ahead();
            if(lineaNegra == false){
                left();
            }else{
                lineaNegra = true;
                left();
                ahead();
                right();
            }
            countD++;
        }else{
            pelotaEncontrada();
            foundPelota = true;
        }
    }
    while (foundPelota == false){
        if(paredAdelante() == true){
            left();
            ahead();
            right();
            ahead();
            right();
            countL++;
        }else{
            pelotaEncontrada();
        } 
    }
    //fuga 
    int pos = countD-countL;
    ahead();
    if(lineaNegra == true){
        if(abs(pos) == 2){
            ahead();
        }
        else if(pos == -3){
            left();
            ahead();
            left();
            ahead();
            right();
            ahead();
        }
        else if(pos == -1 || pos == 3){
            right();
            ahead();
            right();
            ahead();
            left();
            ahead(); 
        }
        else{
            //pos == 0;
            right();
            ahead();
            right();
            ahead();
            ahead();
            right();
            ahead();
            left();
            ahead();
        }
    }else{
        //peor caso
        //pos == 1 -> true
        // or pos == 0
        int it = 0;
        while (lineaNegra == false && foundSalida == false && it<2){
            left();
            ahead();
            if(lineaNegra == false){
                left();
            }else{
                lineaNegra = true;
                right();
            }
            ahead();
            if(lineaNegra == false){
                left();
            }else{
                lineaNegra = true;
                left();
                ahead();
                left();
            }
            if(paredAdelante() == true){
                it++;   
            }else{
                ahead();
                if(getcolor() == 5){ // "rojo
                    foundSalida = true;
                }
            }
            right();
        }
        if (foundSalida == false){
            //tanto 0 como 1 pueden hacer esto
            right();
            ahead();
            right();
            ahead();
            ahead();
            right();
            ahead();
            left();
            if(paredAdelante() == true){
                right();
                ahead();
                right();
                ahead();
                left();
            }
            if(paredAdelante() == false){
                ahead();
            }
        }
    }
}
