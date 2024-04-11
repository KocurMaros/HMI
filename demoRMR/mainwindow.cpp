#include "CKobuki.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QComboBox>
#include <QDebug>
#include <QPainter>
#include <cmath>
#include <memory>
#include <mutex>
#include <qgridlayout.h>
#include <qnamespace.h>

#include "ControllButtons.h"


#include <QImageReader>
#include <QPoint>
#include <math.h>
#include <qdebug.h>
#include "CKobuki.h"

#define BODY_PROGRESS_BAR_POS 3, 2
#define SHORT_MAX 32767
#define TO_RADIANS 3.14159 / 180.0

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
	, m_ui(new Ui::MainWindow)
	, m_connectionLed(new QLed(this))
	, m_robot(nullptr)
	, m_ipaddress(IP_ADDRESSES[0].toStdString())
	, m_motionButtonsVisible(false)
	, m_leftHandedMode(false)
	, m_helpWindow(nullptr)
	, m_useSkeleton(false)
	, m_useTeleView(true)
{
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
	//  cap.open("http://192.168.1.11:8000/stream.mjpg");
	m_ui->setupUi(this);
	m_ui->ipComboBox->addItem(IP_ADDRESSES[0]);
	for (size_t i = 11; i < 15; i++) {
		m_ui->ipComboBox->addItem(IP_ADDRESSES[1] + QString::number(i));
	}
	m_datacounter = 0;
	//  timer = new QTimer(this);
	//	connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
	actIndex = -1;
	useCamera1 = false;
	m_updateSkeletonPicture = 0;

	m_datacounter = 0;
	m_styleSheetEditor = new StyleSheetEditor(this);

	m_ui->topRightLayout->insertWidget(0, m_connectionLed);
	m_ui->pushButton_9->setStyleSheet("background-color: green");

	QImageReader reader = QImageReader(":/img/warning.png");

	m_colisionImage = reader.read();
	if (m_colisionImage.isNull())
		qDebug() << "Error: Cannot load image. " << reader.errorString();
	else
		qDebug() << "Image loaded";
	m_colisionImage = m_colisionImage.scaled(150, 150, Qt::KeepAspectRatio);

	m_mapLoader.loadMap(MAP_PATH, m_mapArea);
}

MainWindow::~MainWindow()
{
	delete m_ui;
}
double MAP(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

QRectF create_border_rect(QRect rect, size_t i)
{
	QRectF border_rect;
	int offset = rect.height() / 40;
	int outside_size = rect.height() / 10;

	if (i == lidarSectors ::FRONT)
		border_rect = QRect(rect.x(), rect.y(), rect.width(), rect.height() / 40);
	else if (i == lidarSectors::FRONT_RIGHT)
		border_rect = QRect(rect.x() + rect.width() - rect.width() / 50, rect.y() + offset, rect.width() / 50, outside_size);
	else if (i == lidarSectors::RIGHT)
		border_rect = QRect(rect.x() + rect.width() - rect.width() / 50, rect.y() + offset + outside_size, rect.width() / 50,
							rect.height() - offset - 2 * outside_size);
	else if (i == lidarSectors::REAR_RIGHT)
		border_rect = QRect(rect.x() + rect.width() - rect.width() / 50, rect.y() + offset + rect.height() - offset - outside_size, rect.width() / 50,
							outside_size);
	else if (i == lidarSectors::LEFT)
		border_rect = QRect(rect.x(), rect.y() + offset + outside_size, rect.width() / 50, rect.height() - offset - 2 * outside_size);
	else if (i == lidarSectors::REAR_LEFT)
		border_rect = QRect(rect.x(), rect.y() + offset + rect.height() - offset - outside_size, rect.width() / 50, outside_size);
	else if (i == lidarSectors::FRONT_LEFT)
		border_rect = QRect(rect.x(), rect.y() + offset, rect.width() / 50, outside_size);
	return border_rect;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
	if (m_useTeleView) {
		paintTeleControl();
	}
	else {
		paintSupervisorControl();
	}
}

void MainWindow::parse_lidar_data(LaserMeasurement laserData, uint16_t *distance)
{
	uint8_t num_of_scans[8] = { 0 };
	for (size_t i = 0; i < 8; i++) {
		m_avgDist[i] = 0;
	}
	for (size_t i = 0; i < laserData.numberOfScans; i++) {
		if (m_copyOfLaserData.Data[i].scanDistance >= lidarDistance::FAR || m_copyOfLaserData.Data[i].scanDistance < 0.1) {
			continue;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::FRONT_B) || m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::FRONT_A)) {
			m_avgDist[lidarSectors::FRONT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::FRONT]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::FRONT_A) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::RIGHT_A)) {
			m_avgDist[lidarSectors::FRONT_RIGHT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::FRONT_RIGHT]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::RIGHT_A) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::RIGHT_B)) {
			m_avgDist[lidarSectors::RIGHT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::RIGHT]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::RIGHT_B) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::REAR_A)) {
			m_avgDist[lidarSectors::REAR_RIGHT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::REAR_RIGHT]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::REAR_A) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::REAR_B)) {
			m_avgDist[lidarSectors::REAR] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::REAR]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::REAR_B) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::LEFT_B)) {
			m_avgDist[lidarSectors::REAR_LEFT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::REAR_LEFT]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::LEFT_B) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::LEFT_A)) {
			m_avgDist[lidarSectors::LEFT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::LEFT]++;
		}
		if (m_copyOfLaserData.Data[i].scanAngle >= (float)(lidarAngles::LEFT_A) && m_copyOfLaserData.Data[i].scanAngle <= (float)(lidarAngles::FRONT_B)) {
			m_avgDist[lidarSectors::FRONT_LEFT] += m_copyOfLaserData.Data[i].scanDistance;
			num_of_scans[lidarSectors::FRONT_LEFT]++;
		}
	}
	for (size_t i = 0; i < 8; i++) {
		m_avgDist[i] /= num_of_scans[i];
		if (m_avgDist[i] < lidarDistance::CLOSE)
			distance[i] = lidarDistance::CLOSE;
		else if (m_avgDist[i] < lidarDistance::MEDIUM)
			distance[i] = lidarDistance::MEDIUM;
		else
			distance[i] = lidarDistance::FAR;
	}
}

void MainWindow::calc_colisions_points(LaserMeasurement laserData, bool *colisions)
{
	const double b = 200.0;

	double d_crit;
	if (m_forwardRobot) {
		for (size_t i = 0; i < laserData.numberOfScans; i++) {
			d_crit = std::abs(b / sin(laserData.Data[i].scanAngle * M_PI / 180.0));
			if (d_crit >= laserData.Data[i].scanDistance && d_crit < lidarDistance::CRITICAL
				&& (laserData.Data[i].scanAngle >= 270.0 || laserData.Data[i].scanAngle <= 90.0) && laserData.Data[i].scanDistance != 0) {
				*colisions = true;
			}
		}
	}
}

void MainWindow::paintTeleControl()
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
	rect = m_ui->frame->geometry(); //ziskate porametre stvorca,do ktoreho chcete kreslit
	rect.translate(0, 37);
	painter.drawRect(rect);

	QRect miniRect;
	if (!m_useSkeleton && !m_motionButtonsVisible) {
		miniRect = m_ui->minLidarFrame->geometry();
		miniRect.translate(0, 37);
		painter.drawRect(miniRect);
	}

	if (useCamera1 == true && actIndex > -1 && !m_reverseRobot) /// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
	{
		drawImageData(painter, rect);
		if (!m_useSkeleton && !m_motionButtonsVisible || m_ui->minLidarFrame->geometry().height() < 50) {
			pero.setWidth(1);
			drawLidarData(painter, pero, miniRect, 70);

			painter.setBrush(Qt::black);
			pero.setColor(Qt::magenta);
			painter.setPen(pero);
			int xrobot = miniRect.width() / 2;
			int yrobot = miniRect.height() / 2;
			int xpolomer = 10;
			int ypolomer = 10;

			painter.drawEllipse(QPoint(miniRect.x() + xrobot, miniRect.y() + yrobot), xpolomer, ypolomer);
			painter.drawLine(miniRect.x() + xrobot, miniRect.y() + yrobot, miniRect.x() + xrobot + xpolomer * cos((360 - 90) * 3.14159 / 180),
							 miniRect.y() + ((yrobot + ypolomer * sin((360 - 90) * 3.14159 / 180))));
		}
	}
	else {
		if (!m_useSkeleton && !m_motionButtonsVisible || m_ui->minLidarFrame->geometry().height() < 50) {
			drawImageData(painter, miniRect, true);
		}
		drawLidarData(painter, pero, rect);

		painter.setBrush(Qt::black);
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

	if (m_updateSkeletonPicture == 1 && m_useSkeleton) {
		m_updateSkeletonPicture = 0;
		inPaintEventProcessSkeleton();
	}
}

void MainWindow::paintSupervisorControl()
{
	QPainter painter(this);
	painter.setBrush(Qt::black);

	QPen pero;
	pero.setStyle(Qt::SolidLine);
	pero.setWidth(3);
	pero.setColor(Qt::green);

	QRect rect(20, 120, 700, 500);
	rect = m_ui->frame->geometry();
	rect.translate(0, 15);
	painter.drawRect(rect);

	for (int i = 0; i < m_mapArea.wall.points.size(); i++) {
		painter.setPen(pero);
		int xmin = rect.width() * (m_mapArea.wall.points[i].point.x - m_mapLoader.minX) / (m_mapLoader.maxX - m_mapLoader.minX);
		int xmax = rect.width() * (m_mapArea.wall.points[(i + 1) % m_mapArea.wall.points.size()].point.x - m_mapLoader.minX)
			/ (m_mapLoader.maxX - m_mapLoader.minX);
		int ymin = rect.height() - rect.height() * (m_mapArea.wall.points[i].point.y - m_mapLoader.minY) / (m_mapLoader.maxY - m_mapLoader.minY);
		int ymax = rect.height()
			- rect.height() * (m_mapArea.wall.points[(i + 1) % m_mapArea.wall.points.size()].point.y - m_mapLoader.minY) / (m_mapLoader.maxY - m_mapLoader.minY);
		painter.drawLine(rect.x() + xmin, rect.y() + ymin, rect.x() + xmax, rect.y() + ymax);
	}

	for (int i = 0; i < m_mapArea.obstacle.size(); i++) {
		for (int j = 0; j < m_mapArea.obstacle[i].points.size(); j++) {
			painter.setPen(pero);
			int xmin = rect.width() * (m_mapArea.obstacle[i].points[j].point.x - m_mapLoader.minX) / (m_mapLoader.maxX - m_mapLoader.minX);
			int xmax = rect.width() * (m_mapArea.obstacle[i].points[(j + 1) % m_mapArea.obstacle[i].points.size()].point.x - m_mapLoader.minX)
				/ (m_mapLoader.maxX - m_mapLoader.minX);
			int ymin = rect.height() - rect.height() * (m_mapArea.obstacle[i].points[j].point.y - m_mapLoader.minY) / (m_mapLoader.maxY - m_mapLoader.minY);
			int ymax = rect.height()
				- rect.height() * (m_mapArea.obstacle[i].points[(j + 1) % m_mapArea.obstacle[i].points.size()].point.y - m_mapLoader.minY)
					/ (m_mapLoader.maxY - m_mapLoader.minY);
			painter.drawLine(rect.x() + xmin, rect.y() + ymin, rect.x() + xmax, rect.y() + ymax);
		}
	}

	pero.setColor(Qt::red);
	painter.setPen(pero);

	double x = getX() * 100 + 50;
	double y = getY() * 100 + 50;

	qDebug() << "x: " << x << " y: " << y;

	int xrobot = rect.width() * (x - m_mapLoader.minX) / (m_mapLoader.maxX - m_mapLoader.minX);
	int yrobot = rect.height() - rect.height() * (y - m_mapLoader.minY) / (m_mapLoader.maxY - m_mapLoader.minY);

	int xpolomer = rect.width() * (20) / (m_mapLoader.maxX - m_mapLoader.minX);
	int ypolomer = rect.height() * (20) / (m_mapLoader.maxY - m_mapLoader.minY);

	painter.drawEllipse(QPoint(rect.x() + xrobot, rect.y() + yrobot), xpolomer, ypolomer);
	painter.drawLine(rect.x() + xrobot, rect.y() + yrobot, rect.x() + xrobot + xpolomer * cos((360 - m_fi * 180. / M_PI) * 3.14159 / 180),
					 rect.y() + (yrobot + ypolomer * sin((360 - getFi() * 180. / M_PI) * 3.14159 / 180)));
}

double MainWindow::getX()
{
	std::scoped_lock<std::mutex> lck(m_odometryLock);
	return m_x;
}

double MainWindow::getY()
{
	std::scoped_lock<std::mutex> lck(m_odometryLock);
	return m_y;
}

double MainWindow::getFi()
{
	std::scoped_lock<std::mutex> lck(m_odometryLock);
	return m_fi;
}

/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
	m_ui->lineEdit_2->setText(QString::number(robotX));
	m_ui->lineEdit_3->setText(QString::number(robotY));
	m_ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
	calculateOdometry(robotdata);

	///tu mozete robit s datami z robota
	/// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
	///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
	/// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
	if (m_instance->count() > 0 || (m_useSkeleton && m_robot != nullptr)) {
		if (forwardspeed == 0 && m_rotationspeed != 0) {
			m_robot->setRotationSpeed(m_rotationspeed);
		}
		else if (forwardspeed != 0 && m_rotationspeed == 0) {
			m_robot->setTranslationSpeed(forwardspeed);
		}
		else if ((forwardspeed != 0 && m_rotationspeed != 0)) {
			m_robot->setArcSpeed(forwardspeed, forwardspeed / m_rotationspeed);
		}
		else {
			m_robot->setTranslationSpeed(0);
		}
	}
	///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

	if (m_datacounter % 5) {
		///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
		// ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
		//ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
		//ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
		/// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
		/// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
		/// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
		///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
		emit uiValuesChanged(getX(), getY(), getFi());
		///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
		/// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
		/// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
	}
	m_datacounter++;

	return 0;
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
	memcpy(&m_copyOfLaserData, &laserData, sizeof(LaserMeasurement));
	//tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
	// ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
	m_updateLaserPicture = 1;
	update(); //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

	return 0;
}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{
	cameraData.copyTo(frame[(actIndex + 1) % 3]); //kopirujem do nasej strukury
	actIndex = (actIndex + 1) % 3;				  //aktualizujem kde je nova fotka
	m_updateLaserPicture = 1;
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
	memcpy(&m_skeleJoints, &skeledata, sizeof(skeleton));
	m_updateSkeletonPicture = 1;
	return 0;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	auto key = event->key();
	switch (key) {
	case Qt::Key_W:
	case Qt::Key_Up:
		forwardspeed = 200;
		m_rotationspeed = 0;
		setRobotDirection();
		m_robot->setTranslationSpeed(forwardspeed);
		break;

	case Qt::Key_S:
	case Qt::Key_Down:
		forwardspeed = -150;
		m_rotationspeed = 0;
		setRobotDirection();
		m_robot->setTranslationSpeed(forwardspeed);
		break;

	case Qt::Key_A:
	case Qt::Key_Left:
		forwardspeed = 0;
		m_rotationspeed = 3.14159 / 2;
		setRobotDirection();
		m_robot->setRotationSpeed(m_rotationspeed);
		break;

	case Qt::Key_D:
	case Qt::Key_Right:
		forwardspeed = 0;
		m_rotationspeed = -3.14159 / 2;
		setRobotDirection();
		m_robot->setRotationSpeed(m_rotationspeed);
		break;

	case Qt::Key_R:
		forwardspeed = 0;
		m_rotationspeed = 0;
		setRobotDirection();
		m_robot->setTranslationSpeed(forwardspeed);
		break;

	case Qt::Key_Escape: {
		if (m_robot->isInEmgStop()) {
			m_robot->setEmgStop(false);
			m_connectionLed->setToConnectedState(QString::fromStdString(m_ipaddress));

			disableAllButtons(false);
			setStyleSheet("");

			return;
		}

		m_robot->setTranslationSpeed(0);
		m_robot->setEmgStop(true);
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
	if (m_robot != nullptr && m_robot->isInEmgStop()) {
		return;
	}

	if (m_connectionLed->isInConnectedState()) {
		qDebug() << "Disconnecting the UI";
		m_robot.reset();

		m_connectionLed->setToDisconnectedState();
		m_ui->pushButton_9->setText("Connect");
		m_ui->pushButton_9->setStyleSheet("");

		m_copyOfLaserData.numberOfScans = 0;

		return;
	}

	QString tmpIP = m_ui->ipComboBox->currentText();
	qDebug() << "Connecting to " << tmpIP;
	m_ipaddress = tmpIP.toStdString();
	qDebug() << "Address " << tmpIP << " " << isIPValid(tmpIP);

	m_robot = std::make_unique<Robot>(m_ipaddress);
	//ziskanie joystickov
	m_instance = QJoysticks::getInstance();
	forwardspeed = 0;
	m_rotationspeed = 0;
	///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
	/// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
	m_robot->setLaserParameters(
		m_ipaddress, 52999, 5299,
		/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/ std::bind(&MainWindow::processThisLidar, this, std::placeholders::_1));
	m_robot->setRobotParameters(m_ipaddress, 53000, 5300, std::bind(&MainWindow::processThisRobot, this, std::placeholders::_1));
	//---simulator ma port 8889, realny robot 8000

	if (m_ipaddress == "127.0.0.1") {
		m_robot->setCameraParameters("http://" + m_ipaddress + ":8889/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
	}
	else {
		m_robot->setCameraParameters("http://" + m_ipaddress + ":8000/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
	}

	m_robot->setSkeletonParameters(m_ipaddress, 23432, 23432, std::bind(&MainWindow::processThisSkeleton, this, std::placeholders::_1));

	///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
	m_robot->robotStart();

	int i = 0;
	for (; i < 3; i++) {
		if (m_robot->isConnected()) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::cout << "waiting for connection" << std::endl;
	}
	if (i == 3) {
		std::cerr << "Connection failed" << std::endl;
		m_robot.reset();
		return;
	}

	m_connectionLed->setToConnectedState(QString::fromStdString(m_ipaddress));
	m_ui->pushButton_9->setText("Disconnect");
	m_ui->pushButton_9->setStyleSheet("background-color: red");

	//tu sa nastartuju vlakna ktore citaju data z lidaru a robota
	connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double)));

	/// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
	/// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
	connect(m_instance, &QJoysticks::axisChanged, [this](const int js, const int axis, const qreal value) {
		if (/*js==0 &&*/ axis == 1) {
			forwardspeed = -value * 300;
		}
		if (/*js==0 &&*/ axis == 0) {
			m_rotationspeed = -value * (3.14159 / 2.0);
		}
	});
}

void MainWindow::on_pushButton_clicked()
{
	if (m_robot == nullptr) {
		return;
	}
	if (useCamera1 == true) {
		useCamera1 = false;

		m_ui->pushButton->setText("use camera");
	}
	else {
		useCamera1 = true;

		m_ui->pushButton->setText("use laser");
	}
}

void MainWindow::on_emgStopButton_clicked()
{
	if (m_robot == nullptr) {
		return;
	}

	if (m_robot->isInEmgStop()) {
		m_robot->setEmgStop(false);
		m_connectionLed->setToConnectedState(QString::fromStdString(m_ipaddress));

		disableAllButtons(false);
		setStyleSheet("");

		return;
	}

	m_robot->setTranslationSpeed(0);
	m_robot->setEmgStop(true);
	m_connectionLed->setToEmgStopState();

	disableAllButtons(true);
	setStyleSheet("background-color: rgba(255,164,0,25)");
	m_reverseRobot = false;
}

void MainWindow::on_bodyControlButton_clicked()
{
	qDebug() << "Body control button clicked. Old: " << m_useSkeleton;
	m_useSkeleton = (m_useSkeleton ? false : true);
	qDebug() << "New: " << m_useSkeleton;

	if (m_useSkeleton) {
		m_bodyProgressBars = new BodyProgressBars(this);

		connect(this, &MainWindow::changeSpeed, m_bodyProgressBars, &BodyProgressBars::setValues);

		m_ui->bodyControlButton->setText("Body Control: on");

		if (m_leftHandedMode) {
			m_controllButtons->addProgressBars(m_bodyProgressBars);
		}
		else {
			m_ui->topGridLayout->addWidget(m_bodyProgressBars, BODY_PROGRESS_BAR_POS);
		}
	}
	else {
		m_ui->bodyControlButton->setText("Body Control: off");

		if (m_leftHandedMode) {
			m_controllButtons->removeProgressBars(m_bodyProgressBars);
		}
		else {
			m_ui->topGridLayout->removeWidget(m_bodyProgressBars);
		}

		m_bodyProgressBars->deleteLater();
	}
}

void MainWindow::on_changeStyleSheet_triggered()
{
	m_styleSheetEditor->show();
	m_styleSheetEditor->activateWindow();
}

void MainWindow::on_actionAdd_motion_buttons_triggered()
{
	if (!m_motionButtonsVisible) {
		m_motionButtonsVisible = true;

		m_controllButtons = new ControllButtons(&m_reverseRobot, &m_forwardRobot, this);
		if (m_leftHandedMode) {
			if (m_useSkeleton) {
				m_ui->topGridLayout->removeWidget(m_bodyProgressBars);
				m_controllButtons->addProgressBars(m_bodyProgressBars);
			}
			else {
				m_ui->topGridLayout->addWidget(m_bodyProgressBars, BODY_PROGRESS_BAR_POS);
			}
			m_ui->topGridLayout->addWidget(m_controllButtons, 1, 2, 1, 1);
			m_controllButtons->switchHand(m_leftHandedMode);
		}
		else {
			m_ui->topGridLayout->addWidget(m_controllButtons, 3, 4, 1, 1);
		}
		update();
		return;
	}

	m_motionButtonsVisible = false;
	m_ui->actionAdd_motion_buttons->setText("Add motion buttons");
	if (m_useSkeleton) {
		m_controllButtons->removeProgressBars(m_bodyProgressBars);
		m_ui->topGridLayout->addWidget(m_bodyProgressBars, BODY_PROGRESS_BAR_POS);
	}
	m_ui->topGridLayout->removeWidget(m_controllButtons);
	m_controllButtons->deleteLater();
}


void MainWindow::on_actionChangeHand_toggled()
{
	m_leftHandedMode = m_ui->actionChangeHand->isChecked();
	m_ui->actionChangeHand->setChecked(m_leftHandedMode);

	if (m_controllButtons == nullptr || m_ui->topGridLayout->indexOf(m_controllButtons) == -1) {
		return;
	}

	m_ui->topGridLayout->removeWidget(m_controllButtons);
	if (m_leftHandedMode) {
		m_ui->topGridLayout->addWidget(m_controllButtons, 1, 2, 1, 1);
		if (m_useSkeleton) {
			m_controllButtons->addProgressBars(m_bodyProgressBars);
			m_ui->topGridLayout->removeWidget(m_bodyProgressBars);
		}
	}
	else {
		if (m_useSkeleton) {
			m_controllButtons->removeProgressBars(m_bodyProgressBars);
			m_ui->topGridLayout->addWidget(m_bodyProgressBars, BODY_PROGRESS_BAR_POS);
		}
		m_ui->topGridLayout->addWidget(m_controllButtons, 3, 4, 1, 1);
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

void MainWindow::on_teleControlButton_clicked()
{
	m_useTeleView = true;
}

void MainWindow::on_supervisorButton_clicked()
{
	m_useTeleView = false;
}

void MainWindow::inPaintEventProcessSkeleton()
{
	double left_zero = -M_PI / 2 - M_PI / 4;
	double right_zero = -M_PI / 4;
	double angle_right = atan2(m_skeleJoints.joints[right_wrist].y - m_skeleJoints.joints[right_elbow].y,
							   m_skeleJoints.joints[right_wrist].x - m_skeleJoints.joints[right_elbow].x);
	double angle_left = atan2(m_skeleJoints.joints[left_wrist].y - m_skeleJoints.joints[left_elbow].y,
							  m_skeleJoints.joints[left_wrist].x - m_skeleJoints.joints[left_elbow].x);
	double speed = 0;
	double rotation = 0;

	if ((m_skeleJoints.joints[left_elbow].x == 0 && m_skeleJoints.joints[left_elbow].y == 0)
		|| (m_skeleJoints.joints[left_wrist].x == 0 && m_skeleJoints.joints[left_wrist].y == 0))
		angle_left = left_zero;
	if ((m_skeleJoints.joints[right_elbow].x == 0 && m_skeleJoints.joints[right_elbow].y == 0)
		|| (m_skeleJoints.joints[right_wrist].x == 0 && m_skeleJoints.joints[right_wrist].y == 0))
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
		speed = MAP(angle_left, left_zero - M_PI / 4, left_zero, -250, 0);
	else if (angle_left > left_zero)
		speed = MAP(angle_left, left_zero, left_zero + M_PI / 4, 0, 250);
	if (angle_right < right_zero)
		rotation = MAP(angle_right, right_zero - M_PI / 4, right_zero, -3.14159 / 4, 0);
	else if (angle_right > right_zero)
		rotation = MAP(angle_right, right_zero, right_zero + M_PI / 4, 0, 3.14159 / 4);
	bool change[2] = { false };
	// cout << "angle_left: " << angle_left << " angle_right: " << angle_right << endl;
	if ((m_prevForwardspeed > 20 || m_prevForwardspeed < 20) && speed == 0) { }
	else if (std::abs(m_prevForwardspeed - speed) > 20) {
		m_prevForwardspeed = speed;
		forwardspeed = speed;
		cout << "Speed: " << forwardspeed << endl;
		change[0] = true;
	}
	if (std::abs(m_prevRotationspeed - rotation) > 0.1) {
		m_prevRotationspeed = rotation;
		m_rotationspeed = rotation;
		// cout << "Rotation: " << rotationspeed << endl;
		change[1] = true;
	}
	if (change[0] || change[1]) {
		change[0] = false;
		change[1] = false;
		emit changeSpeed(forwardspeed, m_rotationspeed);
	}
	setRobotDirection();
}

void MainWindow::setRobotDirection()
{
	if (forwardspeed > 0) {
		m_forwardRobot = true;
		m_reverseRobot = false;
	}
	else if (forwardspeed < 0) {
		m_forwardRobot = false;
		m_reverseRobot = true;
	}
	else {
		m_forwardRobot = false;
		m_reverseRobot = false;
	}
}

void MainWindow::drawLidarData(QPainter &painter, QPen &pen, QRect &rect, int scale)
{
	if (m_updateLaserPicture == 1) ///ak mam nove data z lidaru
	{
		m_updateLaserPicture = 0;
		painter.setPen(pen);
		double min_dist = 10000;
		for (int k = 0; k < m_copyOfLaserData.numberOfScans /*360*/; k++) {
			if (m_reverseRobot) {
				if (m_copyOfLaserData.Data[k].scanAngle <= (float)(lidarAngles::LEFT_B) && m_copyOfLaserData.Data[k].scanAngle >= (float)(lidarAngles::RIGHT_B)) {
					if (min_dist > m_copyOfLaserData.Data[k].scanDistance) {
						min_dist = m_copyOfLaserData.Data[k].scanDistance;
					}
					if (m_copyOfLaserData.Data[k].scanDistance < lidarDistance::CLOSE) {
						painter.setPen(QPen(Qt::red, 3));
					}
					else if (m_copyOfLaserData.Data[k].scanDistance < lidarDistance::MEDIUM) {
						painter.setPen(QPen(Qt::yellow, 3));
					}
					else {
						painter.setPen(QPen(Qt::green, 3));
					}
				}
				else {
					painter.setPen(QPen(QColor(0, 255, 0, 40), 3));
				}
				if (m_ipaddress != "127.0.0.1") {
					CKobuki kobuki;
					if (min_dist < lidarDistance::CLOSE) {
						kobuki.setSound(1000, 1);
					}
					else if (min_dist < lidarDistance::MEDIUM) {
						kobuki.setSound(100, 1);
					}
				}
			}
			else {
				painter.setPen(QPen(Qt::green, 3));
			}

			int dist = m_copyOfLaserData.Data[k].scanDistance / scale; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
			int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - m_copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
				+ rect.topLeft().x(); //prepocet do obrazovky
			int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - m_copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
				+ rect.topLeft().y();  //prepocet do obrazovky
			if (rect.contains(xp, yp)) //ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
				painter.drawEllipse(QPoint(xp, yp), 2, 2);
		}
	}
}

void MainWindow::drawImageData(QPainter &painter, QRect &rect, bool mini)
{
	if (m_robot == nullptr) {
		return;
	}

	QImage image = QImage((uchar *)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step,
						  QImage::Format_RGB888); //kopirovanie cvmat do qimage
	parse_lidar_data(m_copyOfLaserData, m_distanceFromWall);
	calc_colisions_points(m_copyOfLaserData, &m_colisionDetected);

	image = image.scaled(rect.width(), rect.height(), Qt::KeepAspectRatio);
	painter.drawImage(rect, image.rgbSwapped());

	if (m_colisionDetected && !mini) {
		uint16_t width = rect.width();
		uint16_t height = rect.height();

		painter.drawImage(QPoint(width / 2 - m_colisionImage.width() / 2, height / 2 - m_colisionImage.height() / 2), m_colisionImage);
		m_colisionDetected = false;
	}
	else {
		QRectF border_rect;
		QBrush brush;
		for (size_t i = 0; i < 8; i++) {
			if (m_distanceFromWall[i] != lidarDistance::FAR && i != lidarSectors::REAR) {
				border_rect = create_border_rect(rect, i);
				brush.setStyle(Qt::SolidPattern);
				brush.setColor(QColor(
					255, m_distanceFromWall[i] == lidarDistance::CLOSE ? 0 : 255, 0,
					(uint8_t)MAP(m_avgDist[i], (m_distanceFromWall[i] == lidarDistance::MEDIUM) ? (double)lidarDistance::CLOSE : 0.0,
								 (m_distanceFromWall[i] == lidarDistance::MEDIUM) ? (double)lidarDistance::MEDIUM : (double)lidarDistance::CLOSE, 255.0, 30.0)));
				painter.setBrush(brush);
				painter.setPen(Qt::NoPen);
				painter.drawRect(border_rect);
			}
		}
	}
}

void MainWindow::calculateOdometry(const TKobukiData &robotdata)
{
	if (m_robot == nullptr) {
		return;
	}

	int diffLeftEnc = robotdata.EncoderLeft - m_lastLeftEncoder;
	int diffRightEnc = robotdata.EncoderRight - m_lastRightEncoder;

	if (m_lastRightEncoder > 60'000 && robotdata.EncoderRight < 1'000)
		diffRightEnc += SHORT_MAX;
	if (m_lastLeftEncoder > 60'000 && robotdata.EncoderLeft < 1'000)
		diffLeftEnc += SHORT_MAX;

	if (m_lastRightEncoder < 1'000 && robotdata.EncoderRight > 60'000)
		diffRightEnc -= SHORT_MAX;
	if (m_lastLeftEncoder < 1'000 && robotdata.EncoderLeft > 60'000)
		diffLeftEnc -= SHORT_MAX;

	auto leftEncDist = m_robot->tickToMeter * diffLeftEnc;
	auto rightEncDist = m_robot->tickToMeter * diffRightEnc;

	m_lastLeftEncoder = robotdata.EncoderLeft;
	m_lastRightEncoder = robotdata.EncoderRight;

	double l = (rightEncDist + leftEncDist) / 2.0;
	{
		std::scoped_lock<std::mutex> lck(m_odometryLock);
		m_fi = robotdata.GyroAngle / 100. * TO_RADIANS - m_fiCorrection;
		m_x = m_x + l * std::cos(m_fi);
		m_y = m_y + l * std::sin(m_fi);

		if (!m_robotStartupLocation && m_datacounter % 5) {
			m_x = 0;
			m_y = 0;
			m_fiCorrection = m_fi;
			m_robotStartupLocation = true;
		}
	}
}
