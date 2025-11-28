#include "externalcontrolserver.h"



externalControlServer::externalControlServer(QObject *parent)  :
    QTcpServer(parent)//:
  //  serveur(new QTcpServer(this))
{
}

void externalControlServer::startServer(void)
{
    qDebug() << "sMasterController : démarrage du serveur";
    QString message;
    QMessageBox msgBox;

    //this->socketDescriptor = ID;
    // Gestion du serveur
    if (this->isListening())
    {
        msgBox.setText("Serveur en cours de fonctionnement ");
        msgBox.exec();
        return;
    }
    if (!this->listen(QHostAddress::Any, 50885)) // Démarrage du serveur sur toutes les IP disponibles et sur le port 50585
    {
        // Si le serveur n'a pas été démarré correctement
        msgBox.setText("Echec demarrage serveur. ");
        msgBox.exec();

    }
    else
    {

        // Si le serveur a été démarré correctement

        message ="Le serveur de connecté au port : ";
        message.append(QString::number(this->serverPort()));
        message.append(".\n");
        message.append("Des clients peuvent maintenant se connecter.");
        msgBox.setText(message);
        msgBox.exec();
       connect(this, SIGNAL(newConnection()), this, SLOT(nouvelleConnexion()));

    }

    tailleMessage = 0;

}

//void externalControlServer::stopServer(void)
//{
//    qDebug() << "sMaster Controller : Arret du controleur";
//    serveur->close();
//    delete serveur;


//}

void externalControlServer::nouvelleConnexion(void)
{
    //QString message;
    envoyerATous(tr("<em>Un nouveau client vient de se connecter</em>"));
    QTcpSocket *nouveauClient = this->nextPendingConnection();
    clients << nouveauClient;
    connect(nouveauClient, SIGNAL(readyRead()), this, SLOT(donneesRecues()));
    connect(nouveauClient, SIGNAL(disconnected()), this, SLOT(deconnexionClient()));

/*    QtControllerClient *client = new QtControllerClient(this);
    client->SetSocket(this->socketDescriptor());*/

}
void externalControlServer::donneesRecues()
{
//    QString buffer;

//    // 1 : on reçoit un paquet (ou un sous-paquet) d'un des clients
//    // On détermine quel client envoie le message (recherche du QTcpSocket du client)
    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket == 0) // Si par hasard on n'a pas trouvé le client à l'origine du signal, on arrête la méthode
        return;
    // Si tout va bien, on continue : on récupère le message
    QDataStream in(socket);
    if (tailleMessage == 0) // Si on ne connaît pas encore la taille du message, on essaie de la récupérer
    {
        if (socket->bytesAvailable() < (int)sizeof(quint16)) // On n'a pas reçu la taille du message en entier
            return;
        in >> tailleMessage; // Si on a reçu la taille du message en entier, on la récupère

    }
    //    // Si on connaît la taille du message, on vérifie si on a reçu le message en entier
    if (socket->bytesAvailable() < tailleMessage) // Si on n'a pas encore tout reçu, on arrête la méthode
        return;
    // Si ces lignes s'exécutent, c'est qu'on a reçu tout le message : on peut le récupérer !
    QString message;
    in >> message;

    // 2 : on renvoie le message à tous les clients
    envoyerATous(message);
    // 3 : remise de la taille du message à 0 pour permettre la réception des futurs messages
    tailleMessage = 0;
}
void externalControlServer::deconnexionClient()
{
    envoyerATous(tr("<em>Le client vient de se déconnecter</em>"));
    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket == 0)
        return ;
    clients.removeOne(socket);
    socket->deleteLater();
}
void externalControlServer::envoyerATous(const QString &message)
{

    // Préparation du paquet
    QByteArray paquet;
    QDataStream out(&paquet, QIODevice::WriteOnly);
    out << (quint16) 0; // On écrit 0 au début du paquet pour réserver la place pour écrire la taille
    out << message; // On ajoute le message à la suite
    out.device()->seek(0); // On se replace au début du paquet
    out << (quint16) (paquet.size() - sizeof(quint16)); // On écrase le 0 qu'on avait réservé par la longueur du message
    // Envoi du paquet préparé à tous les clients connectés au serveur*/
    for (int i=0; i < clients.size();i++)
        clients[i]->write(paquet);
}
// si un client demande une connexion
//void externalControlServer :: demande_connexion()
// {
//    emit vers_IHM_connexion(); // on envoie un signal à l'IHM
//    // on crée une nouvelle socket pour ce client
//    clientConnection = nextPendingConnection();
//    // si on reçoit des données, le slot lecture() est appelé
//    QObject:: connect(clientConnection, SIGNAL(readyRead()),
//    this, SLOT(lecture()));
//}
//void externalControlServer ::lecture()
//{
//    QString ligne;
//    while(clientConnection->canReadLine())    // tant qu'on peut lire sur la socket
//    {
//        ligne = clientConnection->readLine(); // on lit une ligne
//        emit vers_IHM_texte(ligne);           // on l'envoie à l'IHM
//    }
//    QTextStream texte(clientConnection);      // création d'un flux pour écrire dans la socket
//    texte << "message reçu" << endl;          // message à envoyer au client
//}
