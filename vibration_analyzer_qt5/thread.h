#ifndef THREAD_H
#define THREAD_H

#include <QThread>
#include <QtDebug>

class Thread : public QThread
{
    Q_OBJECT

public:
    explicit Thread(QObject *parent = 0);

private:
    void run();
};

#endif // THREAD_H
