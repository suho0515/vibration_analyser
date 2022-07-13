#include "thread.h"

Thread::Thread(QObject *parent) :
    QThread(parent)
{

}
void Thread::run()
{
    int i = 0;
    while(1)
    {
        i++;
        emit Send(i);
    }
}
