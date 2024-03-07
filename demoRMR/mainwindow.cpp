#include "CKobuki.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QComboBox>
#include <QDebug>
#include <QPainter>
#include <cmath>
#include <memory>

static QString IP_ADDRESSES[2] {"127.0.0.1", "192.168.1."};
// 11-15

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
	, m_ipaddress(IP_ADDRESSES[0].toStdString())
{
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	 //192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
							 //  cap.open("http://192.168.1.11:8000/stream.mjpg");
	ui->setupUi(this);
	ui->ipComboBox->addItem(IP_ADDRESSES[0]);
	for (size_t i = 11; i < 15; i++) {
		ui->ipComboBox->addItem(IP_ADDRESSES[1]+QString::number(i));
	}
	datacounter = 0;
	//  timer = new QTimer(this);
	//	connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
	actIndex = -1;
	useCamera1 = false;

	datacounter = 0;
	m_styleSheetEditor = new StyleSheetEditor(this);

	ui->topRightLayout->insertWidget(0, m_connectionLed);
	ui->pushButton_9->setStyleSheet("background-color: green");
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

	if (useCamera1 == true && actIndex > -1) /// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
	{
		std::cout << actIndex << std::endl;
		QImage image = QImage((uchar *)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step,
							  QImage::Format_RGB888); //kopirovanie cvmat do qimage
		painter.drawImage(rect, image.rgbSwapped());
	}
	else {
		if (updateLaserPicture == 1) ///ak mam nove data z lidaru
		{
			updateLaserPicture = 0;

			painter.setPen(pero);
			//teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
			//   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
			for (int k = 0; k < copyOfLaserData.numberOfScans /*360*/; k++) {
				int dist = copyOfLaserData.Data[k].scanDistance / 20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
				int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().x(); //prepocet do obrazovky
				int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().y();  //prepocet do obrazovky
				if (rect.contains(xp, yp)) //ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
					painter.drawEllipse(QPoint(xp, yp), 2, 2);
			}
		}
	}
	if(updateSkeletonPicture==1 )
	{
		painter.setPen(Qt::red);
		for(int i=0;i<75;i++)
		{
			int xp=rect.width()-rect.width() * skeleJoints.joints[i].x+rect.topLeft().x();
			int yp= (rect.height() *skeleJoints.joints[i].y)+rect.topLeft().y();
			if(rect.contains(xp,yp))
				painter.drawEllipse(QPoint(xp, yp),2,2);
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

int MainWindow::processThisSkeleton(skeleton skeledata)
{
	memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

	updateSkeletonPicture=1;
	return 0;
}

void MainWindow::on_pushButton_9_clicked() //start button
{
	if (robot != nullptr && robot->isInEmgStop()) {
		return;
	}

	if (m_connectionLed->isInConnectedState()) {

		qDebug() << "Disconnecting the UI";
		robot.reset();

		m_connectionLed->setToDisconnectedState();
		ui->pushButton_9->setText("Connect");
		ui->pushButton_9->setStyleSheet("");

		copyOfLaserData.numberOfScans = 0;

		return;
	}

	QString tmpIP = ui->ipComboBox->currentText();
	qDebug() << "Connecting to " << tmpIP;
	m_ipaddress = tmpIP.toStdString();
	qDebug() << "Address " << tmpIP << " " << isIPValid(tmpIP);

	robot = std::make_unique<Robot>(m_ipaddress);
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

	if (m_ipaddress == "127.0.0.1") {
		robot->setCameraParameters("http://" + m_ipaddress + ":8889/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
	}
	else {
		robot->setCameraParameters("http://" + m_ipaddress + ":8000/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
	}

	robot->setSkeletonParameters(m_ipaddress, 23432,23432,std::bind(&MainWindow::processThisSkeleton,this,std::placeholders::_1));

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
		robot.reset();
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
}

void MainWindow::on_pushButton_3_clicked() //back
{
	if (robot == nullptr) {
		return;
	}
	robot->setTranslationSpeed(-250);
}

void MainWindow::on_pushButton_6_clicked() //left
{
	if (robot == nullptr) {
		return;
	}
	robot->setRotationSpeed(3.14159 / 2);
}

void MainWindow::on_pushButton_5_clicked() //right
{
	if (robot == nullptr) {
		return;
	}
	robot->setRotationSpeed(-3.14159 / 2);
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
}

void MainWindow::on_changeStyleSheet_triggered()
{
	m_styleSheetEditor->show();
	m_styleSheetEditor->activateWindow();
}
