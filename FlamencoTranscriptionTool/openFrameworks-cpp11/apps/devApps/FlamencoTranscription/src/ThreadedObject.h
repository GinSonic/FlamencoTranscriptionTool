//
//  ThreadedObject.h
//  testCpp11
//
//  Created by Nadine Kroher on 04.04.14.
//
//

#ifndef testCpp11_ThreadedObject_h
#define testCpp11_ThreadedObject_h
void threadedFunction(){
    while( isThreadRunning() != 0 ){
        if( lock() ){
            if ( dirThread == true ) { // !!! problem here !!!
                cout << "\n---Dir Thread running---\n";
                dirThread = false;
            }
            unlock();
            ofSleepMillis(50);
        }
    }


#endif
