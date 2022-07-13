#include "thread.h"

Thread::Thread(QObject *parent) :
    QThread(parent)
{

}
void Thread::run()
{
    while(1)
    {
        qDebug() << "thread";
    }
}
