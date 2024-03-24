#include "CKobuki.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QComboBox>
#include <QDebug>
#include <QPainter>
#include <cmath>
#include <memory>
#include <qgridlayout.h>
#include <qnamespace.h>

#include "ControllButtons.h"


#include <QImageReader>
#include <QPoint>
#include <math.h>
#include <qdebug.h>
#include "CKobuki.h"

// 11-15
static QString IP_ADDRESSES[2] { "127.0.0.1", "192.168.1." };
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
	, m_motionButtonsVisible(false)
	, m_leftHandedMode(false)
	, m_helpWindow(nullptr)
{
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
	//  cap.open("http://192.168.1.11:8000/stream.mjpg");
	ui->setupUi(this);
	ui->ipComboBox->addItem(IP_ADDRESSES[0]);
	for (size_t i = 11; i < 15; i++) {
		ui->ipComboBox->addItem(IP_ADDRESSES[1] + QString::number(i));
	}
	datacounter = 0;
	//  timer = new QTimer(this);
	//	connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
	actIndex = -1;
	useCamera1 = false;
	updateSkeletonPicture = 0;

	datacounter = 0;
	m_styleSheetEditor = new StyleSheetEditor(this);

	ui->topRightLayout->insertWidget(0, m_connectionLed);
	ui->pushButton_9->setStyleSheet("background-color: green");

	QImageReader reader = QImageReader(":/img/warning.png");

	colision_image = reader.read();
	if (colision_image.isNull())
		qDebug() << "Error: Cannot load image. " << reader.errorString();
	else
		qDebug() << "Image loaded";
	colision_image = colision_image.scaled(150, 150, Qt::KeepAspectRatio);
}

MainWindow::~MainWindow()
{
	delete ui;
}
double MAP(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

QRectF create_border_rect(QRect rect, size_t i)
{
	QRectF border_rect;
	if (i == 0)
		border_rect = QRect(rect.x(), rect.y(), rect.width(), rect.height() / 20);
	else if (i == 1)
		border_rect = QRect(rect.x() + rect.width() - rect.width() / 50, rect.y(), rect.width() / 50, rect.height());
	else if (i == 3) 
		border_rect = QRect(rect.x(), rect.y(), rect.width() / 50, rect.height());
	return border_rect;
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
		calc_colisions_points(copyOfLaserData, &colisionDetected);

		painter.drawImage(rect, image.rgbSwapped());

		uint16_t width = rect.width();
		uint16_t height = rect.height();
		if (colisionDetected) {
			painter.drawImage(QPoint(width / 2 - colision_image.width() / 2, height / 2 - colision_image.height() / 2), colision_image);
			colisionDetected = false;
		}
		else {
			QRectF border_rect;
			QBrush brush;
			for (size_t i = 0; i < 4; i++) {
				if (distanceFromWall[i] != lidarDistance::FAR){
					border_rect = create_border_rect(rect,  i);
					if (distanceFromWall[i] == lidarDistance::MEDIUM) {
						brush.setStyle(Qt::SolidPattern);
						brush.setColor(QColor(255, 255, 0, (uint8_t)MAP(copyOfLaserData.Data[i].scanDistance, 
												distanceFromWall[i] == lidarDistance::CLOSE ? (double)lidarDistance::CLOSE : 0, (double)distanceFromWall[i], 255.0, 0.0)));
						painter.setBrush(brush);
						if (i != 2)
							painter.drawRect(border_rect);
					}
				}
			}
		}
	}else {
		if (updateLaserPicture == 1) ///ak mam nove data z lidaru
		{
			updateLaserPicture = 0;
			painter.setPen(pero);
			int den = 5;
			double min_dist = 10000;
			for (int k = 0; k < copyOfLaserData.numberOfScans /*360*/; k++) {
				if(reverse_robot){
					if (copyOfLaserData.Data[k].scanAngle <= (float)(lidarDirection::REVERSE_LEFT)
						&& copyOfLaserData.Data[k].scanAngle >= (float)(lidarDirection::REVERSE_RIGHT)) {
						if (min_dist > copyOfLaserData.Data[k].scanDistance)
							min_dist = copyOfLaserData.Data[k].scanDistance;
						if (copyOfLaserData.Data[k].scanDistance < lidarDistance::CLOSE)
							painter.setPen(QPen(Qt::red, 3));
						else if (copyOfLaserData.Data[k].scanDistance < lidarDistance::MEDIUM)
							painter.setPen(QPen(Qt::yellow, 3));
						else
							painter.setPen(QPen(Qt::green, 3));
					}
					else 
						painter.setPen(QPen(QColor(0, 255, 0, 40), 3));
					if (m_ipaddress != "127.0.0.1") {
						CKobuki kobuki;
						if (min_dist < lidarDistance::CLOSE) {
							kobuki.setSound(1000, 1);
						}
						else if (min_dist < lidarDistance::MEDIUM) {
							kobuki.setSound(100, 1);
						}
					}
				}else
					painter.setPen(QPen(Qt::green, 3));

				int dist = copyOfLaserData.Data[k].scanDistance / 20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
				int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().x(); //prepocet do obrazovky
				int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().y();  //prepocet do obrazovky
				if (rect.contains(xp, yp)) //ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
					painter.drawEllipse(QPoint(xp, yp), 2, 2);
			}
		}
		pero.setColor(Qt::magenta);
		painter.setPen(pero);
		int xrobot = rect.width() / 2;
		int yrobot = rect.height() / 2;
		int xpolomer = 20;
		int ypolomer = 20;

		painter.drawEllipse(QPoint(rect.x() + xrobot, rect.y() + yrobot), xpolomer, ypolomer);
		painter.drawLine(rect.x() + xrobot, rect.y() + yrobot, rect.x() + xrobot + xpolomer * cos((360 - 90) * 3.14159 / 180),
							rect.y() + ((yrobot + ypolomer * sin((360 - 90) * 3.14159 / 180))));
	}
	if(updateSkeletonPicture == 1)
	{
		updateSkeletonPicture = 0;
		inPaintEventProcessSkeleton();
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
	if (instance->count() > 0 || (updateSkeletonPicture && robot != nullptr)) {
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
	// Disable all the buttons.
	QList<QPushButton *> buttons = findChildren<QPushButton *>();
	for (auto button : buttons) {
		if (button->objectName() == "emgStopButton" || button->objectName() == "pushButton") {
			continue;
		}

		button->setDisabled(disable);
	}

	// Disable the combo box for choosing the IP address.
	QComboBox *comboBox = findChild<QComboBox *>();
	comboBox->setDisabled(disable);
}

bool MainWindow::isIPValid(const QString &ip)
{
	QRegularExpression ipRegex("^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
							   "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
							   "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
							   "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");

	return ipRegex.match(ip).hasMatch();
}

int MainWindow::processThisSkeleton(skeleton skeledata)
{
	memcpy(&skeleJoints, &skeledata, sizeof(skeleton));
	updateSkeletonPicture = 1;
	return 0;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	if (robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	auto key = event->key();
	switch (key) {
	case Qt::Key_W:
	case Qt::Key_Up:
		forwardspeed = 300;
		rotationspeed = 0;
		setRobotDirection();
		robot->setTranslationSpeed(forwardspeed);
		break;

	case Qt::Key_S:
	case Qt::Key_Down:
		forwardspeed = -300;
		rotationspeed = 0;
		setRobotDirection();
		robot->setTranslationSpeed(forwardspeed);
		break;

	case Qt::Key_A:
	case Qt::Key_Left:
		forwardspeed = 0;
		rotationspeed = 3.14159 / 2;
		setRobotDirection();
		robot->setRotationSpeed(rotationspeed);
		break;

	case Qt::Key_D:
	case Qt::Key_Right:
		forwardspeed = 0;
		rotationspeed = -3.14159 / 2;
		setRobotDirection();
		robot->setRotationSpeed(rotationspeed);
		break;

	case Qt::Key_R:
		forwardspeed = 0;
		rotationspeed = 0;
		setRobotDirection();
		robot->setTranslationSpeed(forwardspeed);
		break;

	case Qt::Key_Escape: {
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
		break;
	}

	default:
		qDebug() << "Key not recognized";
		break;
	}
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

	robot->setSkeletonParameters(m_ipaddress, 23432, 23432, std::bind(&MainWindow::processThisSkeleton, this, std::placeholders::_1));

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

void MainWindow::on_bodyControlButton_clicked()
{
	qDebug() << "Body control button clicked";
	updateSkeletonPicture = (updateSkeletonPicture ? 0 : 1);
}

void MainWindow::on_changeStyleSheet_triggered()
{
	m_styleSheetEditor->show();
	m_styleSheetEditor->activateWindow();
}

void MainWindow::parse_lidar_data(LaserMeasurement laserData, uint16_t *distance)
{
	double avg_dist[4] = { 0 };
	uint8_t num_of_scans[4] = { 0 };
	for (size_t i = 0; i < laserData.numberOfScans; i++) {
		if (copyOfLaserData.Data[i].scanDistance < 0.1)
			continue;
		if (copyOfLaserData.Data[i].scanAngle >= (float)(lidarDirection::FRONT_LEFT)
			|| copyOfLaserData.Data[i].scanAngle <= (float)(lidarDirection::FRONT_RIGHT) && copyOfLaserData.Data[i].scanDistance < lidarDistance::FAR) { // front side
			avg_dist[0] += laserData.Data[i].scanDistance;
			num_of_scans[0]++;
		}
		if (copyOfLaserData.Data[i].scanAngle >= (float)(lidarDirection::FRONT_RIGHT_1) && copyOfLaserData.Data[i].scanAngle <= (float)(lidarDirection::RIGHT)
			&& copyOfLaserData.Data[i].scanDistance < lidarDistance::FAR) { //right side
			// cout << laserData.Data[i].scanDistance << " " << laserData.Data[i].scanAngle << " " << laserData.Data[i].scanQuality << endl;
			avg_dist[1] += laserData.Data[i].scanDistance;
			num_of_scans[1]++;
		}
		if (copyOfLaserData.Data[i].scanAngle >= (float)(lidarDirection::LEFT) && copyOfLaserData.Data[i].scanAngle <= (float)(lidarDirection::FRONT_LEFT_1)
			&& copyOfLaserData.Data[i].scanDistance < lidarDistance::FAR) { //left side
			avg_dist[3] += laserData.Data[i].scanDistance;
			num_of_scans[3]++;
		}
	}
	for (size_t i = 0; i < 4; i++) {
		avg_dist[i] /= num_of_scans[i];
		if (avg_dist[i] < lidarDistance::CLOSE)
			distance[i] = lidarDistance::CLOSE;
		else if (avg_dist[i] < lidarDistance::MEDIUM)
			distance[i] = lidarDistance::MEDIUM;
		else
			distance[i] = lidarDistance::FAR;
	}
}

void MainWindow::calc_colisions_points(LaserMeasurement laserData, bool *colisions)
{
	const double b = 250.0;

	double d_crit;
	cout << forward_robot << endl;
	if (forward_robot) {
		for (size_t i = 0; i < laserData.numberOfScans; i++) {
			d_crit = std::abs(b / sin(laserData.Data[i].scanAngle * M_PI / 180.0));
			if (d_crit >= laserData.Data[i].scanDistance && laserData.Data[i].scanDistance < lidarDistance::CLOSE
				&& (laserData.Data[i].scanAngle >= 270.0 || laserData.Data[i].scanAngle <= 90.0) && laserData.Data[i].scanDistance != 0) {
				*colisions = true;
			}
		}
	}
}

void MainWindow::on_actionAdd_motion_buttons_triggered()
{
	if (!m_motionButtonsVisible) {
		m_motionButtonsVisible = true;

		m_controllButtons = new ControllButtons(&reverse_robot, &forward_robot, this);
		if (m_leftHandedMode) {
			ui->topGridLayout->addWidget(m_controllButtons, 1, 2, 1, 1);
			m_controllButtons->switchHand(m_leftHandedMode);
		}
		else {
			ui->topGridLayout->addWidget(m_controllButtons, 4, 4, 1, 1);
		}
		update();
		return;
	}

	m_motionButtonsVisible = false;
	ui->actionAdd_motion_buttons->setText("Add motion buttons");
	ui->topGridLayout->removeWidget(m_controllButtons);
	m_controllButtons->deleteLater();
}


void MainWindow::on_actionChangeHand_toggled()
{
	m_leftHandedMode = ui->actionChangeHand->isChecked();
	ui->actionChangeHand->setChecked(m_leftHandedMode);

	if (m_controllButtons == nullptr || ui->topGridLayout->indexOf(m_controllButtons) == -1) {
		return;
	}

	ui->topGridLayout->removeWidget(m_controllButtons);
	if (m_leftHandedMode) {
		ui->topGridLayout->addWidget(m_controllButtons, 1, 2, 1, 1);
	}
	else {
		ui->topGridLayout->addWidget(m_controllButtons, 4, 4, 1, 1);
	}

	m_controllButtons->switchHand(m_leftHandedMode);
	update();
}

void MainWindow::on_actionShowHelp_triggered()
{
	m_helpWindow = new HelpWindow(this);
	m_helpWindow->setWindowFlags(Qt::Window);
	m_helpWindow->show();

	connect(m_helpWindow->ui.closeButton, &QPushButton::clicked, [this]() { m_helpWindow->close(); });
}


void MainWindow::inPaintEventProcessSkeleton(){
	double left_zero = -M_PI / 2 - M_PI / 4;
	double right_zero = -M_PI / 4;
	double angle_right = atan2(skeleJoints.joints[right_wrist].y - skeleJoints.joints[right_elbow].y,
								skeleJoints.joints[right_wrist].x - skeleJoints.joints[right_elbow].x);
	double angle_left = atan2(skeleJoints.joints[left_wrist].y - skeleJoints.joints[left_elbow].y,
								skeleJoints.joints[left_wrist].x - skeleJoints.joints[left_elbow].x);
	double speed = 0;
	double rotation = 0;

	if ((skeleJoints.joints[left_elbow].x == 0 && skeleJoints.joints[left_elbow].y == 0)
		|| (skeleJoints.joints[left_wrist].x == 0 && skeleJoints.joints[left_wrist].y == 0))
		angle_left = left_zero;
	if ((skeleJoints.joints[right_elbow].x == 0 && skeleJoints.joints[right_elbow].y == 0)
		|| (skeleJoints.joints[right_wrist].x == 0 && skeleJoints.joints[right_wrist].y == 0))
		angle_right = right_zero;
	if (angle_left < left_zero - M_PI / 4 || angle_left > 0)
		angle_left = left_zero - M_PI / 4;
	else if (angle_left > left_zero + M_PI / 4)
		angle_left = left_zero + M_PI / 4;

	if (angle_right < right_zero - M_PI / 4)
		angle_right = right_zero - M_PI / 4;
	else if (angle_right > right_zero + M_PI / 4)
		angle_right = right_zero + M_PI / 4;

	if (angle_left < left_zero)
		speed = MAP(angle_left, left_zero - M_PI / 4, left_zero, -300, 0);
	else if (angle_left > left_zero)
		speed = MAP(angle_left, left_zero, left_zero + M_PI / 4, 0, 300);
	if (angle_right < right_zero)
		rotation = MAP(angle_right, right_zero - M_PI / 4, right_zero, -3.14159 / 4, 0);
	else if (angle_right > right_zero)
		rotation = MAP(angle_right, right_zero, right_zero + M_PI / 4, 0, 3.14159 / 4);
	forwardspeed = speed;
	rotationspeed = rotation;
	setRobotDirection();
}
void MainWindow::setRobotDirection(){
	if(forwardspeed > 0){
		forward_robot = true;
		reverse_robot = false;
	}
	else if(forwardspeed < 0){
		forward_robot = false;
		reverse_robot = true;
	}
	else{
		forward_robot = false;
		reverse_robot = false;
	}
}
