#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QImageReader>
#include <QPoint>
#include <math.h>
#include <qdebug.h>
#include "CKobuki.h"
///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, m_connectionLed(new QLed(this))
	, robot(nullptr)
{
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	m_ipaddress = "127.0.0.1"; //192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
							 //  cap.open("http://192.168.1.11:8000/stream.mjpg");
	ui->setupUi(this);
	datacounter = 0;
	//  timer = new QTimer(this);
	//	connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
	actIndex = -1;
	useCamera1 = false;
	datacounter = 0;
	m_styleSheetEditor = new StyleSheetEditor(this);

	ui->topRightLayout->insertWidget(0, m_connectionLed);
	ui->pushButton_9->setStyleSheet("background-color: green");

	QImageReader reader = QImageReader(":/img/attention.png");
	attention_image = reader.read();
	if(attention_image.isNull()) 
		qDebug() << "Error: Cannot load image. " << reader.errorString();
	else
		qDebug() << "Image loaded";
	attention_image = attention_image.scaled(150, 150, Qt::KeepAspectRatio);
	QImageReader reader1 = QImageReader(":/img/warning.png");

	warning_image = reader1.read();
	if(warning_image.isNull())
		qDebug() << "Error: Cannot load image. " << reader.errorString();
	else
		qDebug() << "Image loaded";
	warning_image = warning_image.scaled(150, 150, Qt::KeepAspectRatio);
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
	/// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
	painter.setBrush(Qt::black); //cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
	QPen pero;
	pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
	pero.setWidth(3);			  //hrubka pera -3pixely
	pero.setColor(Qt::green);	  //farba je zelena
	QRect rect;
	rect = ui->frame->geometry(); //ziskate porametre stvorca,do ktoreho chcete kreslit
	rect.translate(0, 37);
	painter.drawRect(rect);

	if (useCamera1 == true && actIndex > -1 && !reverse_robot) /// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
	{
		QImage image = QImage((uchar *)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step,
							  QImage::Format_RGB888); //kopirovanie cvmat do qimage
        parse_lidar_data(copyOfLaserData, distanceFromWall);
		calc_colisions_points(copyOfLaserData, colisionDetected);
		
		painter.drawImage(rect, image.rgbSwapped());
		
		QPoint dest_pos;
		uint16_t width = rect.width();
		uint16_t height = rect.height();
		uint16_t x,y;
		if(colisionDetected[0] || colisionDetected[1]){
			painter.drawImage(QPoint(width - warning_image.width(), height/2 - warning_image.height()/2),attention_image);
			colisionDetected[0] = false;
			colisionDetected[1] = false;
		}
		for(size_t i=0;i<4;i++){
			if(distanceFromWall[i] != lidarDistance::FAR){
				if(i == 0){
					x = width/2 - attention_image.width()/2;
					y = height/2 - attention_image.height()/2;
				}else if(i == 1){
					x = width/2+width/4 - attention_image.width()/2;
					y = height/2 - attention_image.height()/2;
				}else if(i == 3){ 
					x = width/2-width/4 - attention_image.width()/2;
					y = height/2 - attention_image.height()/2;
				}
				dest_pos = QPoint(x, y);
			}
			if(distanceFromWall[i] == lidarDistance::MEDIUM){
				if(i != 2)
					painter.drawImage(dest_pos,warning_image);
			}
			if(distanceFromWall[i] == lidarDistance::CLOSE){
				if(i != 2)
					painter.drawImage(dest_pos,attention_image);
			}
		}	
	}
	else {
		if(reverse_robot){
			updateLaserPicture = 0;
			double min_dist = 10000;
			painter.setPen(pero);
			int den = 5;
			for (int k = 0; k < copyOfLaserData.numberOfScans /*360*/; k++) {
				if(	copyOfLaserData.Data[k].scanAngle <= (float)(lidarDirection::REVERSE_LEFT) && 
					copyOfLaserData.Data[k].scanAngle >= (float)(lidarDirection::REVERSE_RIGHT)){
					if(min_dist > copyOfLaserData.Data[k].scanDistance)
						min_dist = copyOfLaserData.Data[k].scanDistance;
					if(copyOfLaserData.Data[k].scanDistance < lidarDistance::CLOSE)
						painter.setPen(QPen(Qt::red, 3));
					else if(copyOfLaserData.Data[k].scanDistance < lidarDistance::MEDIUM)
						painter.setPen(QPen(Qt::yellow, 3));
					else
						painter.setPen(QPen(Qt::green, 3));
				}else{
					painter.setPen(QPen(QColor(0,255,0,40), 3));
				}
				// CKobuki kobuki;
				// if(min_dist < lidarDistance::CLOSE){
				// 	kobuki.setSound(1000,1);
				// }else if(min_dist < lidarDistance::MEDIUM){
				// 	kobuki.setSound(100,1);
				// }

				int dist = copyOfLaserData.Data[k].scanDistance / 20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
				int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().x(); //prepocet do obrazovky
				int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().y();  //prepocet do obrazovky
				if (rect.contains(xp, yp)) //ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
					painter.drawEllipse(QPoint(xp, yp), 2, 2);
			}
			pero.setColor(Qt::magenta);
			painter.setPen(pero);
			int xrobot=rect.width()/2;
			int yrobot=rect.height()/2;
			int xpolomer=20;
			int ypolomer=20;

			painter.drawEllipse(QPoint(rect.x()+xrobot,rect.y()+yrobot),xpolomer,ypolomer);
			painter.drawLine(rect.x()+xrobot,rect.y()+yrobot,rect.x()+xrobot+xpolomer*cos((360-90)*3.14159/180),rect.y()+((yrobot+ypolomer*sin((360-90)*3.14159/180))));
		}else if (updateLaserPicture == 1) ///ak mam nove data z lidaru
		{
			updateLaserPicture = 0;

			painter.setPen(pero);
			int den = 5;
			for (int k = 0; k < copyOfLaserData.numberOfScans /*360*/; k++) {
				if(copyOfLaserData.Data[k].scanAngle >= (float)(330) - (float)(lidarDirection::THRESHOLD/den) &&
				   copyOfLaserData.Data[k].scanAngle <= (float)(330) + (float)(lidarDirection::THRESHOLD/den)){
					painter.setPen(QPen(Qt::red, 3));
				}
				else
					painter.setPen(QPen(Qt::green, 3));
				
				int dist = copyOfLaserData.Data[k].scanDistance / 20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
				int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().x(); //prepocet do obrazovky
				int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().y();  //prepocet do obrazovky
				if (rect.contains(xp, yp)) //ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
					painter.drawEllipse(QPoint(xp, yp), 2, 2);
			}
			pero.setColor(Qt::magenta);
			painter.setPen(pero);
			int xrobot=rect.width()/2;
			int yrobot=rect.height()/2;
			int xpolomer=20;
			int ypolomer=20;

			painter.drawEllipse(QPoint(rect.x()+xrobot,rect.y()+yrobot),xpolomer,ypolomer);
			painter.drawLine(rect.x()+xrobot,rect.y()+yrobot,rect.x()+xrobot+xpolomer*cos((360-90)*3.14159/180),rect.y()+((yrobot+ypolomer*sin((360-90)*3.14159/180))));
		}
	}
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
	ui->lineEdit_2->setText(QString::number(robotX));
	ui->lineEdit_3->setText(QString::number(robotY));
	ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
	///tu mozete robit s datami z robota
	/// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
	///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
	/// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
	if (instance->count() > 0) {
		if (forwardspeed == 0 && rotationspeed != 0)
			robot->setRotationSpeed(rotationspeed);
		else if (forwardspeed != 0 && rotationspeed == 0)
			robot->setTranslationSpeed(forwardspeed);
		else if ((forwardspeed != 0 && rotationspeed != 0))
			robot->setArcSpeed(forwardspeed, forwardspeed / rotationspeed);
		else
			robot->setTranslationSpeed(0);
	}
	///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

	if (datacounter % 5) {
		///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
		// ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
		//ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
		//ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
		/// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
		/// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
		/// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
		///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
		emit uiValuesChanged(robotdata.EncoderLeft, 11, 12);
		///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
		/// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
		/// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
	}
	datacounter++;

	return 0;
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
	memcpy(&copyOfLaserData, &laserData, sizeof(LaserMeasurement));
	//tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
	// ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
	updateLaserPicture = 1;
	update(); //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


	return 0;
}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{
	cameraData.copyTo(frame[(actIndex + 1) % 3]); //kopirujem do nasej strukury
	actIndex = (actIndex + 1) % 3;				  //aktualizujem kde je nova fotka
	updateLaserPicture = 1;
	return 0;
}

void MainWindow::disableAllButtons(bool disable)
{
	QList<QPushButton *> buttons = findChildren<QPushButton *>();
	for (auto button : buttons) {
		if (button->objectName() == "emgStopButton"
			|| button->objectName() == "pushButton") {
			continue;
		}

		button->setDisabled(disable);
	}
}

bool MainWindow::isIPValid(const QString &ip)
{
	QRegularExpression ipRegex(
		"^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
		"(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
		"(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
		"(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");

	return ipRegex.match(ip).hasMatch();
}

void MainWindow::on_pushButton_9_clicked() //start button
{
	if (robot != nullptr && robot->isInEmgStop()) {
		return;
	}

	if (m_connectionLed->isInConnectedState()) {

		delete robot;
		robot = nullptr;

		m_connectionLed->setToDisconnectedState();
		ui->pushButton_9->setText("Connect");
		ui->pushButton_9->setStyleSheet("");

		copyOfLaserData.numberOfScans = 0;

		return;
	}

	QString tmpIP = ui->ipHintLineEdit->text();
	if (tmpIP.isEmpty()) {
		tmpIP = "127.0.0.1";
		ui->ipHintLineEdit->setText(tmpIP);
	}

	if (!isIPValid(tmpIP)) {
		ui->ipHintLineEdit->clear();
		QMessageBox::warning(this, "Invalid IP", tmpIP + " is an invalid IP address");
		return;
	}

	m_ipaddress = tmpIP.toStdString();

	robot = new Robot();
	//ziskanie joystickov
	instance = QJoysticks::getInstance();
	forwardspeed = 0;
	rotationspeed = 0;

	///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
	/// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
	robot->setLaserParameters(
		m_ipaddress, 52999, 5299,
		/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/ std::bind(&MainWindow::processThisLidar, this, std::placeholders::_1));
	robot->setRobotParameters(m_ipaddress, 53000, 5300, std::bind(&MainWindow::processThisRobot, this, std::placeholders::_1));
	//---simulator ma port 8889, realny robot 8000
	robot->setCameraParameters("http://" + m_ipaddress + ":8889/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));

	///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
	robot->robotStart();

	int i = 0;
	for (; i < 3; i++) {
		if (robot->isConnected()) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::cout << "waiting for connection" << std::endl;
	}
	if (i == 3) {
		std::cerr << "Connection failed" << std::endl;
		delete robot;
		return;
	}

	m_connectionLed->setToConnectedState(QString::fromStdString(m_ipaddress));
	ui->pushButton_9->setText("Disconnect");
	ui->pushButton_9->setStyleSheet("background-color: red");

	//tu sa nastartuju vlakna ktore citaju data z lidaru a robota
	connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double)));

	/// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
	/// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
	connect(instance, &QJoysticks::axisChanged, [this](const int js, const int axis, const qreal value) {
		if (/*js==0 &&*/ axis == 1) {
			forwardspeed = -value * 300;
		}
		if (/*js==0 &&*/ axis == 0) {
			rotationspeed = -value * (3.14159 / 2.0);
		}
	});
}

void MainWindow::on_pushButton_2_clicked() //forward
{
	//pohyb dopredu
	if (robot == nullptr) {
		return;
	}
	robot->setTranslationSpeed(500);
	reverse_robot = false;
}

void MainWindow::on_pushButton_3_clicked() //back
{
	if (robot == nullptr) {
		return;
	}
	robot->setTranslationSpeed(-250);
	reverse_robot = true;
}

void MainWindow::on_pushButton_6_clicked() //left
{
	if (robot == nullptr) {
		return;
	}
	robot->setRotationSpeed(3.14159 / 2);
	reverse_robot = false;
}

void MainWindow::on_pushButton_5_clicked() //right
{
	if (robot == nullptr) {
		return;
	}
	robot->setRotationSpeed(-3.14159 / 2);
	reverse_robot = false;
}

void MainWindow::on_pushButton_4_clicked() //stop
{
	if (robot == nullptr) {
		return;
	}
	robot->setTranslationSpeed(0);
}

void MainWindow::on_pushButton_clicked()
{
	if (robot == nullptr) {
		return;
	}
	if (useCamera1 == true) {
		useCamera1 = false;

		ui->pushButton->setText("use camera");
	}
	else {
		useCamera1 = true;

		ui->pushButton->setText("use laser");
	}
}

void MainWindow::on_emgStopButton_clicked()
{
	if (robot == nullptr) {
		return;
	}

	if (robot->isInEmgStop()) {
		robot->setEmgStop(false);
		m_connectionLed->setToConnectedState(QString::fromStdString(m_ipaddress));

		disableAllButtons(false);
		setStyleSheet("");

		return;
	}

	robot->setTranslationSpeed(0);
	robot->setEmgStop(true);
	m_connectionLed->setToEmgStopState();

	disableAllButtons(true);
	setStyleSheet("background-color: rgba(255,164,0,25)");
	reverse_robot = false;
}

void MainWindow::on_changeStyleSheet_triggered()
{
	m_styleSheetEditor->show();
	m_styleSheetEditor->activateWindow();
}

void MainWindow::parse_lidar_data(LaserMeasurement laserData, uint16_t *distance){
	double avg_dist[4] = {0};
	uint8_t num_of_scans[4] = {0};

    for(size_t i = 0; i < laserData.numberOfScans; i++){
		if(	copyOfLaserData.Data[i].scanAngle >= (float)(lidarDirection::FRONT_LEFT) ||
			copyOfLaserData.Data[i].scanAngle <= (float)(lidarDirection::FRONT_RIGHT)){// front side
            avg_dist[0] += laserData.Data[i].scanDistance;
			num_of_scans[0]++;
        }
		if(	copyOfLaserData.Data[i].scanAngle >= (float)(lidarDirection::FRONT_RIGHT) &&
			copyOfLaserData.Data[i].scanAngle <= (float)(lidarDirection::REAR_RIGHT)){ //right side
			avg_dist[1] += laserData.Data[i].scanDistance;
			num_of_scans[1]++;
		}
		if(	copyOfLaserData.Data[i].scanAngle >= (float)(lidarDirection::REAR_LEFT) &&
			copyOfLaserData.Data[i].scanAngle <= (float)(lidarDirection::FRONT_LEFT)){//left side
			avg_dist[3] += laserData.Data[i].scanDistance;
			num_of_scans[3]++;
		}
    }
	for(size_t i = 0; i < 4; i++){
		avg_dist[i] /= num_of_scans[i];
		if(avg_dist[i] < lidarDistance::CLOSE)
			distance[i] = lidarDistance::CLOSE;
		else if(avg_dist[i] < lidarDistance::MEDIUM)
			distance[i] = lidarDistance::MEDIUM;
		else
			distance[i] = lidarDistance::FAR;
	}
}

void MainWindow::calc_colisions_points(LaserMeasurement laserData,bool *colisions){

	static const double dist = 400;

	for(size_t i = 0; i < laserData.numberOfScans; i++){

		if(	copyOfLaserData.Data[i].scanAngle >= (float)(0) ||
			copyOfLaserData.Data[i].scanAngle <= (float)(90)){
			if(laserData.Data[i].scanDistance < dist || laserData.Data[i].scanDistance < dist){
				colisions[0] = true;
			}
		}
		if(	copyOfLaserData.Data[i].scanAngle >= (float)(360) ||
			copyOfLaserData.Data[i].scanAngle <= (float)(270)){
			if(laserData.Data[i].scanDistance < dist || laserData.Data[i].scanDistance < dist){
				colisions[1] = true;
			}
		}
	}
}