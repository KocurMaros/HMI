#include "CKobuki.h"
#include "ControllButtons.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <algorithm>
#include <cmath>
#include <cmath>
#include <memory>
#include <mutex>

#include <qnamespace.h>
#include <QComboBox>
#include <QDebug>
#include <QGridLayout>
#include <QImageReader>
#include <QMessageBox>
#include <QPainter>
#include <QPoint>
#include <QPushButton>

#define BODY_PROGRESS_BAR_POS 3, 2
#define SHORT_MAX 65'535
#define TO_RADIANS 3.14159 / 180.0
#define DRAG_N_DROP_RANGE 10

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
	, m_odometryThread(new QThread(this))
	, m_positionTracker(new PositionTracker())
	, m_connectionLed(new QLed(this))
	, m_robot(nullptr)
	, m_ipaddress(IP_ADDRESSES[0].toStdString())
	, m_motionButtonsVisible(false)
	, m_leftHandedMode(false)
	, m_helpWindow(nullptr)
	, m_useSkeleton(false)
    , m_ObjectDetection(new ObjectDetection(this))
	, m_lastLeftEncoder(0)
	, m_lastRightEncoder(0)
	, m_endPosition(nullptr)
	, m_userMode(UserMode::Supervisor)
	, m_dragNDrop(false)
	, m_floodPlannerThread(new QThread(this))
{
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
	//  cap.open("http://192.168.1.11:8000/stream.mjpg");
	m_ui->setupUi(this);
	m_ui->ipComboBox->addItem(IP_ADDRESSES[0]);
	for (size_t i = 11; i <= 15; i++) {
		m_ui->ipComboBox->addItem(IP_ADDRESSES[1] + QString::number(i));
	}

	m_datacounter = 0;
	actIndex = -1;
	useCamera1 = false;
	m_updateSkeletonPicture = 0;

	m_datacounter = 0;
	m_styleSheetEditor = new StyleSheetEditor(this);

	connect(this, &MainWindow::batteryLevel, m_connectionLed, &QLed::on_batterLevel_received);
	m_ui->topRightLayout->insertWidget(0, m_connectionLed);
	m_ui->pushButton_9->setStyleSheet("background-color: green");
	// m_ObjectDetection = ObjectDetection();

	QImageReader reader = QImageReader(":/img/warning.png");

	m_colisionImage = reader.read();
	if (m_colisionImage.isNull())
		qDebug() << "Error: Cannot load image. " << reader.errorString();
	else
		qDebug() << "Image loaded";
	m_colisionImage = m_colisionImage.scaled(150, 150, Qt::KeepAspectRatio);
	qRegisterMetaType<cv::Mat>("cv::Mat");  //barz zaregistrovat typ, ktory chcete posielat cez signal slot
	qRegisterMetaType<cv::Point>("cv::Point"); 

	if (m_userMode == UserMode::Supervisor) {
		on_actionSupervisor_triggered();
	}
	else {
		on_actionTelecontrol_triggered();
	}

	m_mapLoader = std::make_shared<MapLoader>(this, m_ui->frame->width(), m_ui->frame->height());
	// m_mapLoader->loadMap(MAP_PATH);
	connect(this, &MainWindow::positionResults, m_positionTracker, &PositionTracker::on_positionResults_handle, Qt::QueuedConnection);
	connect(m_positionTracker, &PositionTracker::resultsReady, this, &MainWindow::on_resultsReady_updateUi, Qt::QueuedConnection);

	m_floodPlanner = std::make_shared<FloodPlanner>("/home/fildo7525/Documents/STU/LS/HMI/zadania/z1/HMI/map.txt");

	connect(this, &MainWindow::requestPath, m_floodPlanner.get(), &FloodPlanner::on_requestPath_plan, Qt::QueuedConnection);
	connect(m_floodPlanner.get(), &FloodPlanner::pathPlanned, this, &MainWindow::handlePath, Qt::QueuedConnection);

	m_floodPlanner->moveToThread(m_floodPlannerThread);
	m_floodPlannerThread->start();

	setMouseTracking(true);
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
	if (m_userMode == UserMode::Telecontrol) {
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
	for (size_t i = 0; i < laserData.numberOfScans; i++) {
		d_crit = std::abs(b / sin(laserData.Data[i].scanAngle * M_PI / 180.0));
		if (d_crit >= laserData.Data[i].scanDistance && d_crit < lidarDistance::CRITICAL
			&& (laserData.Data[i].scanAngle >= 270.0 || laserData.Data[i].scanAngle <= 90.0) && laserData.Data[i].scanDistance != 0) {
			*colisions = true;
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

	QRect rect;
	rect = m_ui->frame->geometry();
	rect.translate(0, 37);
	painter.drawRect(rect);

	painter.setPen(pero);

	auto walls = m_mapLoader->walls();
	for (const auto &wall : walls) {
		auto min = wall.start;
		auto max = wall.end;
		painter.drawLine(min, max);
	}

	pero.setColor(Qt::red);
	painter.setPen(pero);

	double x = getX() * 100 + 50;
	double y = getY() * 100 + 50;

	double xrobot = rect.width() * (x - m_mapLoader->minX) / (m_mapLoader->maxX - m_mapLoader->minX);
	double yrobot = rect.height() - rect.height() * (y - m_mapLoader->minY) / (m_mapLoader->maxY - m_mapLoader->minY);

	int xpolomer = rect.width() * (20) / (m_mapLoader->maxX - m_mapLoader->minX);
	int ypolomer = rect.height() * (20) / (m_mapLoader->maxY - m_mapLoader->minY);

	QPointF robotPos(rect.x() + xrobot, rect.y() + yrobot);
	painter.drawEllipse(robotPos, xpolomer, ypolomer);
	painter.drawLine(rect.x() + xrobot, rect.y() + yrobot, rect.x() + xrobot + xpolomer * cos((360 - m_fi * 180. / M_PI) * 3.14159 / 180),
					 rect.y() + (yrobot + ypolomer * sin((360 - getFi() * 180. / M_PI) * 3.14159 / 180)));

	bool isInCollision = false;

	auto lineColorSetter = [this, &isInCollision,  &pero, &painter](const QPointF &start, const QPointF &end) {
		if (m_mapLoader->isLineInCollision(start, end)) {
			pero.setColor(Qt::red);
			painter.setPen(pero);
			isInCollision = true;
		}
		else {
			pero.setColor(Qt::white);
			painter.setPen(pero);
		}
	};
	if(m_draw_c){
		m_draw_c = false;
		m_draw_c_was = true;
		double cm2pixels = rect.width()*5.0 / (m_mapLoader->maxX - m_mapLoader->minX);
		pero.setColor(Qt::red);
		painter.setPen(pero);
		object_pos_x = rect.x() + xrobot + m_objectOnMap.x;
		object_pos_y = rect.y() + yrobot - m_objectOnMap.y;
		if(object_pos_x < rect.x()+10)
			object_pos_x = rect.x() + xrobot + m_objectOnMap.x+30;
		if(object_pos_x > rect.x()+rect.width()-10)
			object_pos_x = rect.x() + xrobot + m_objectOnMap.x-30;
		if(object_pos_y < rect.y()+10)
			object_pos_y = rect.y() + yrobot - m_objectOnMap.y+30;
		if(object_pos_y > rect.y()+rect.height()-10)
			object_pos_y = rect.y() + yrobot - m_objectOnMap.y-30;

		if (QPointF(object_pos_x, object_pos_y) != robotPos) {
			painter.drawEllipse(QPoint(object_pos_x, object_pos_y), 10, 10);
		}
		// std::cout << "x: " << rect.x() + xrobot + m_objectOnMap.x << " y: " <<  rect.y() + yrobot - m_objectOnMap.y << std::endl;
		// std::cout << "rect width: " << rect.width() << " rect height: " << rect.height() << std::endl;
	}
	if(m_draw_c_was){
		pero.setColor(Qt::red);
		painter.setPen(pero);
		painter.drawEllipse(QPoint(object_pos_x, object_pos_y), 10, 10);
		// std::cout << "x: " << object_pos_x << " y: " << object_pos_y << std::endl;
		// std::cout << "rect width: " << rect.width() << " rect height: " << rect.height() << std::endl;
	}
	if(isInCollision){
		QBrush brush;
		brush.setStyle(Qt::SolidPattern);
		brush.setColor(Qt::red);
		painter.setBrush(brush);
		painter.setPen(Qt::NoPen);

		painter.drawRect(QRect(rect.x(), rect.y(), rect.width() / 50, rect.height()));	//lavy
		painter.drawRect(QRect(rect.x()+rect.width()- rect.width()/50, rect.y(), rect.width()/50, rect.height())); //pravy
		painter.drawRect(QRect(rect.x(), rect.y(), rect.width(), rect.height()/50));	//horny
		painter.drawRect(QRect(rect.x(), rect.y()+rect.height()- rect.height() / 50, rect.width(), rect.height() / 50)); //dolny
	}
	for (size_t i = 0; i < m_transitionPoints.size(); i++) {
		if (m_transitionPoints.size() == 0) {
			break;
		}

		if (i == 0) {
			lineColorSetter(robotPos, m_transitionPoints[i]);
			painter.drawLine(QLineF(robotPos, m_transitionPoints[i]));
		}
		else {
			lineColorSetter(m_transitionPoints[i - 1], m_transitionPoints[i]);
			painter.drawLine(m_transitionPoints[i - 1], m_transitionPoints[i]);
		}

		pero.setColor(Qt::yellow);
		painter.setPen(pero);
		painter.drawEllipse(m_transitionPoints[i], 3, 3);
	}

	pero.setColor(Qt::cyan);
	for (size_t i = 0; i < m_specialPoints.size(); i++) {
		painter.setPen(pero);
		painter.drawEllipse(m_specialPoints[i], 3, 3);
	}

	pero.setColor(Qt::white);
	painter.setPen(pero);
	if (m_endPosition) {
		if (m_transitionPoints.size() > 0) {
			lineColorSetter(m_transitionPoints.back(), *m_endPosition);
			painter.drawLine(QLineF(m_transitionPoints.back(), *m_endPosition));
		}
		else if (!m_goingHome) {
			lineColorSetter(robotPos, *m_endPosition);
			painter.drawLine(QLineF(robotPos, *m_endPosition));
		}
	}

	m_inCollision = isInCollision;
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

void MainWindow::bodyControlTeleview()
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

void MainWindow::bodyControlSupervisor()
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	if (m_endPosition == nullptr) {
		return;
	}

	if (m_inCollision) {
		QMessageBox::warning(this, "Collision", "Line is in collision");
		return;
	}


	QVector<QPointF> points;
	qDebug() << "Transition points: " << m_transitionPoints;
	std::transform(m_transitionPoints.begin(), m_transitionPoints.end(), std::back_inserter(points),
				   [this](const QPointF &point) { return m_mapLoader->toWorldPoint(point); });

	points.push_back(m_mapLoader->toWorldPoint(*m_endPosition));
	// points.push_back(*m_endPosition);
	qDebug() << "Points: " << points;
	auto [distanceToTarget, angleToTarget] = calculateTrajectoryTo(points.back());

	emit arcResultsReady(distanceToTarget, angleToTarget, std::move(points));
}

void MainWindow::pushButtonTeleview()
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
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

void MainWindow::pushButtonSupervisor()
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	m_transitionPoints.clear();
	m_endPosition.reset();
	emit moveForward(0);
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
	if (m_robot == nullptr || m_userMode == UserMode::Telecontrol) {
		qDebug() << "Robot is not connected or teleoperation is not enabled";
		return;
	}

	bool clickedInFrame = m_ui->frame->geometry().contains(event->pos());
	if (!clickedInFrame) {
		qDebug() << "Clicked outside the frame";
		return;
	}

	QPointF line = createLineParams(event->pos());

	auto x = getX() * 100 + 50;
	auto y = getY() * 100 + 50;
	auto startPoint = (m_transitionPoints.size() == 0 ? m_mapLoader->toMapPoint({ x, y }) : m_transitionPoints.back());

	if (m_mapLoader->isLineInCollision(startPoint, event->pos()) && event->button() != Qt::MiddleButton) {
		qDebug() << "Line is in collision";
		QMessageBox::warning(this, "Collision", "Line is in collision");
		return;
	}

	QPoint pos = event->pos();
	QPointF *point = std::find_if(m_transitionPoints.begin(), m_transitionPoints.end(), [&event, &pos](const QPointF &point){
		return (pos - point).manhattanLength() <= DRAG_N_DROP_RANGE;
	});
	if (event->button() == Qt::RightButton) {
		if (point != m_transitionPoints.end()) {
			m_transitionPoints.erase(point);
		}
		else {
			m_transitionPoints.push_back(pos);
		}
	}
	else if (event->button() == Qt::LeftButton) {
		if (point != m_transitionPoints.end()) {
			m_specialPoints.push_back(*point);
		}
		else {
			m_endPosition = std::make_shared<QPointF>(pos);
		}
	}
	else if (event->button() == Qt::MiddleButton) {
		m_dragNDrop = true;
	}
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event) 
{
	if (m_dragNDrop && event->button() == Qt::MiddleButton) {
		m_dragNDrop = false;
	}
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	QPoint pos = event->pos();

	if (m_dragNDrop) {
		QPointF *point = m_transitionPoints.end();
		if ((*m_endPosition - pos).manhattanLength() <= DRAG_N_DROP_RANGE) {
			point = m_endPosition.get();
		}
		else {
			point = std::find_if(m_transitionPoints.begin(), m_transitionPoints.end(), [&event, &pos](const QPointF &point){
				return (pos - point).manhattanLength() <= DRAG_N_DROP_RANGE;
			});
		}

		if (point != m_transitionPoints.end()) {
			*point = pos;
		}
	}
}

QPointF MainWindow::createLineParams(const QPointF &p)
{
	QPointF line;
	// Compute slope (a)
	if (getX() != p.x()) {
		auto x = (p.y() - getY()) / (p.x() - getX());
		line.setX(x);
	}
	else {
		// If the line is vertical, slope is infinity, so set a to a large value
		line.setY(1e9);
	}
	// Compute intercept (b)
	line.ry() = getY() - line.x() * getX();
	return line;
}

void MainWindow::on_actionTelecontrol_triggered()
{
	m_userMode = UserMode::Telecontrol;

	m_ui->actionAdd_motion_buttons->setDisabled(false);
	m_ui->bodyControlButton->setText("Body Control: off");
	m_ui->pushButton->setText("󰄀   Use camera");

	m_ui->topRightLayout->removeWidget(m_loadMapButton);
	m_loadMapButton->deleteLater();

	m_ui->topRightLayout->removeWidget(m_returnHomeButton);
	m_returnHomeButton->deleteLater();

	for(auto &var : m_supervisorConnections) {
		disconnect(var);
	}

	update();
}

void MainWindow::on_actionSupervisor_triggered()
{
	if (m_motionButtonsVisible) {
		on_actionAdd_motion_buttons_triggered();
	}
	m_ui->actionAdd_motion_buttons->setDisabled(true);
	m_ui->bodyControlButton->setText("  Execute");
	m_ui->pushButton->setText("  Clear points");

	m_loadMapButton = new QPushButton("󰦄  Load map", this);
	m_loadMapButton->setStyleSheet(m_ui->bodyControlButton->styleSheet());
	m_loadMapButton->setMinimumSize(m_ui->bodyControlButton->minimumSize());
	m_loadMapButton->setSizePolicy(m_ui->bodyControlButton->sizePolicy());
	m_supervisorConnections.push_back(connect(m_loadMapButton, &QPushButton::clicked, this, &MainWindow::openFileDialog));
	m_ui->topRightLayout->addWidget(m_loadMapButton, 0, Qt::AlignHCenter);

	m_returnHomeButton = new QPushButton("  Return Home", this);
	m_returnHomeButton->setStyleSheet(m_ui->bodyControlButton->styleSheet());
	m_returnHomeButton->setMinimumSize(m_ui->bodyControlButton->minimumSize());
	m_returnHomeButton->setSizePolicy(m_ui->bodyControlButton->sizePolicy());
	m_supervisorConnections.push_back(connect(m_returnHomeButton, &QPushButton::clicked, this, &MainWindow::returnHomeActivated));
	m_ui->topRightLayout->addWidget(m_returnHomeButton, 0, Qt::AlignHCenter);

	m_userMode = UserMode::Supervisor;

	update();
}

void MainWindow::openFileDialog()
{
	// Open a file dialog and get the selected file
	QString filePath = QFileDialog::getOpenFileName(this, "Open File", "../", "Text Files (*.txt)");

	// You can use the selected file path for further processing if needed
	if (!filePath.isEmpty()) {
		// Do something with the selected file path
		// For example, print the file path to the console
		qDebug() << "Selected file:" << filePath;
		m_mapLoader->loadMap(filePath);
		update();
	}
}

/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
	m_ui->lineEdit_2->setText(QString::number(robotX));
	m_ui->lineEdit_3->setText(QString::number(robotY));
	m_ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::on_rtc_removePoint()
{
	if (m_transitionPoints.size() == 0) {
		return;
	}

	auto point = m_transitionPoints.front();
	auto pos = std::find_if(m_specialPoints.begin(), m_specialPoints.end(), [&point](const QPointF &specialPoint){
		return specialPoint == point;
	});
	if (pos != m_specialPoints.end()) {
		m_specialPoints.erase(pos);
	}

	m_transitionPoints.pop_front();
}

void MainWindow::handlePath(QVector<QPointF> path)
{
	path.push_back(m_dockPosition);
	auto [distance, angle] = calculateTrajectoryTo(m_dockPosition);

	qDebug() << "Path: " << path;
	m_goingHome = true;
	m_endPosition = std::make_shared<QPointF>(path.back());
	emit arcResultsReady(distance, angle, path);
}


void MainWindow::returnHomeActivated()
{
	QPointF start(m_x, m_y);
	qDebug() << "Returning home from: " << start << " to: " << m_dockPosition;
	pushButtonSupervisor();
	emit requestPath(start, m_dockPosition);
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
	calculateOdometry(robotdata);
	int battery = 100 * (robotdata.Battery - 155.) / (167. - 155.);
	emit batteryLevel(battery);

	if (m_instance->count() > 0 || (m_useSkeleton && m_robot != nullptr)) {
		if (forwardspeed == 0 && m_rotationspeed != 0) {
			emit changeRotation(m_rotationspeed);
		}
		else if (forwardspeed != 0 && m_rotationspeed == 0) {
			emit moveForward(forwardspeed);
		}
		else if ((forwardspeed != 0 && m_rotationspeed != 0)) {
			emit changeArc(forwardspeed, m_rotationspeed);
		}
		else {
			emit moveForward(0);
		}
	}

	if (m_datacounter % 5) {
		emit uiValuesChanged(m_x, m_y, m_fi);
	}

	return 0;
}

int MainWindow::processThisLidar(LaserMeasurement laserData)
{
	memcpy(&m_copyOfLaserData, &laserData, sizeof(LaserMeasurement));
	m_updateLaserPicture = 1;
	if(m_detection_update_lidar){
        m_detection_update_lidar = false;
		emit randomSignalObjectDetectionCircleOtherThreadRandom(m_objectDetected);
    }
	update();

	calc_colisions_points(m_copyOfLaserData, &m_colisionDetected);
	if (m_colisionDetected) {
		qDebug() << "Collision detected";
		QMessageBox::warning(this, "Collision", "Collision detected");
		m_transitionPoints.clear();
		m_specialPoints.clear();
		emit moveForward(0);
		m_colisionDetected = false;
	}

	return 0;
}

int MainWindow::processThisCamera(cv::Mat cameraData)
{
	cameraData.copyTo(frame[(actIndex + 1) % 3]);
	actIndex = (actIndex + 1) % 3;
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

int MainWindow::processThisSkeleton(skeleton skeledata)
{
	memcpy(&m_skeleJoints, &skeledata, sizeof(skeleton));
	m_updateSkeletonPicture = 1;
	return 0;
}

QPair<double, double> MainWindow::calculateTrajectory()
{
	// Get current position and orientation (actual values)
	auto [distanceToTarget, angleToTarget] = calculateTrajectoryTo(*m_endPosition);

	return { distanceToTarget, angleToTarget };
}

QPair<double, double> MainWindow::calculateTrajectoryTo(const QPointF &point)
{
	// Get current position and orientation (actual values)
	double current_x = 0;
	double current_y = 0;

	{
		std::scoped_lock<std::mutex> lck(m_odometryLock);
		current_x = m_x;
		current_y = m_y;
	}

	// Calculate angle to target
	double angleToTarget = std::atan2(point.y() - current_y, point.x() - current_x);
	double distanceToTarget = std::sqrt(std::pow(point.y() - current_y, 2) + std::pow(point.x() - current_x, 2));

	return { distanceToTarget, angleToTarget };
}

double MainWindow::finalRotationError()
{
	auto [dir, rot] = calculateTrajectory();
	double diff, fi;
	{
		std::scoped_lock<std::mutex> lck(m_odometryLock);
		fi = m_fi;
	}

	if (fi > PI / 2 && rot < -PI / 2) {
		fi -= 2 * PI;
	}
	if (fi < -PI / 2 && rot > PI / 2) {
		fi += 2 * PI;
	}

	diff = fi - rot;

	return -diff;
}

double MainWindow::localRotationError(const QPointF &point)
{
	auto [dir, rot] = calculateTrajectoryTo(point);
	double diff, fi;
	{
		std::scoped_lock<std::mutex> lck(m_odometryLock);
		fi = m_fi;
	}

	diff = fi - rot;

	if (diff > PI) {
		diff -= 2 * PI;
	}
	if (diff < -PI) {
		diff += 2 * PI;
	}

	return -diff;
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
		emit moveForward(forwardspeed);
		break;

	case Qt::Key_S:
	case Qt::Key_Down:
		forwardspeed = -150;
		m_rotationspeed = 0;
		setRobotDirection();
		emit moveForward(forwardspeed);
		break;

	case Qt::Key_A:
	case Qt::Key_Left:
		forwardspeed = 0;
		m_rotationspeed = 3.14159 / 2;
		setRobotDirection();
		emit changeRotation(m_rotationspeed);
		break;

	case Qt::Key_D:
	case Qt::Key_Right:
		forwardspeed = 0;
		m_rotationspeed = -3.14159 / 2;
		setRobotDirection();
		emit changeRotation(m_rotationspeed);
		break;

	case Qt::Key_R:
		forwardspeed = 0;
		m_rotationspeed = 0;
		setRobotDirection();
		emit moveForward(forwardspeed);
		break;

	case Qt::Key_Escape: {
		if (m_robot->isInEmgStop()) {
			m_robot->setEmgStop(false);
			m_connectionLed->setToConnectedState(QString::fromStdString(m_ipaddress));

			disableAllButtons(false);
			setStyleSheet("");

			return;
		}

		emit moveForward(0);
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
	if (!m_mapLoader->isLoaded()) {
		QMessageBox::warning(this, "Warning", "Map is not loaded. Please load the map first.");
		return;
	}

	if (m_connectionLed->isInConnectedState()) {
		if (m_robot->isInEmgStop()) {
			return;
		}
		m_transitionPoints.clear();
		m_endPosition.reset();

		for (auto &var : m_rtcConnections) {
			disconnect(var);
		}
		m_rtcConnections.clear();

		qDebug() << "Disconnecting the UI";
		emit moveForward(0);

		m_connectionLed->setToDisconnectedState();
		m_ui->pushButton_9->setText("Connect");
		m_ui->pushButton_9->setStyleSheet("");

		m_copyOfLaserData.numberOfScans = 0;
		m_trajectoryController.reset();
		m_robot.reset();

		return;
	}

	if (m_robot != nullptr && m_robot->isInEmgStop()) {
		return;
	}

	// Object for managing the robot speed interactions.
	QString tmpIP = m_ui->ipComboBox->currentText();
	m_ipaddress = tmpIP.toStdString();

	qDebug() << "Connecting the UI to robot at IP address: " << m_ipaddress.c_str();
	m_robot.reset(new Robot(m_ipaddress));

	m_trajectoryController = std::make_shared<RobotTrajectoryController>(m_robot, this);

	m_rtcConnections.push_back(
		connect(this, &MainWindow::moveForward, m_trajectoryController.get(), &RobotTrajectoryController::onMoveForwardMove, Qt::QueuedConnection));
	m_rtcConnections.push_back(
		connect(this, &MainWindow::changeRotation, m_trajectoryController.get(), &RobotTrajectoryController::onChangeRotationRotate, Qt::QueuedConnection));
	m_rtcConnections.push_back(
		connect(this, &MainWindow::changeArc, m_trajectoryController.get(), &RobotTrajectoryController::onMoveArcMove, Qt::QueuedConnection));

	m_rtcConnections.push_back(
		connect(this, &MainWindow::arcResultsReady, m_trajectoryController.get(), &RobotTrajectoryController::handleArcResults, Qt::QueuedConnection));
	m_rtcConnections.push_back(
		connect(m_trajectoryController.get(), &RobotTrajectoryController::removePoint, this, &MainWindow::on_rtc_removePoint, Qt::QueuedConnection));

	m_positionTracker->moveToThread(m_odometryThread);
	m_odometryThread->start();

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
    connect(this, &MainWindow::haffTransform, m_ObjectDetection, &ObjectDetection::detectObjects, Qt::QueuedConnection);
	connect(m_ObjectDetection,&ObjectDetection::on_circleDetected, this, &MainWindow::updateLidarCircle , Qt::QueuedConnection);

	connect(this,&MainWindow::randomSignalObjectDetectionCircleOtherThreadRandom, this, &MainWindow::calculePositionOfObject , Qt::QueuedConnection);
	
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
	start_pressed = true;
	// m_ObjectDetection.detectObjects(frame[actIndex]);
}

void MainWindow::on_pushButton_clicked()
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	if (m_userMode == UserMode::Telecontrol) {
		pushButtonTeleview();
	}
	else {
		pushButtonSupervisor();
	}
}

void MainWindow::on_emgStopButton_clicked()
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
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
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	if (m_userMode == UserMode::Telecontrol) {
		bodyControlTeleview();
	}
	else {
		bodyControlSupervisor();
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

void MainWindow::on_resultsReady_updateUi(double x, double y, double fi)
{
	if (!m_robotStartupLocation && m_datacounter % 5) {
		x = 0;
		y = 0;
		m_fiCorrection = fi;
		m_robotStartupLocation = true;
	}

	{
		std::scoped_lock<std::mutex> lck(m_odometryLock);
		m_x = x;
		m_y = y;
		m_fi = fi - m_fiCorrection;
	}

	if (std::abs(x) < 0.1 && std::abs(y) < 0.1 && m_goingHome) {
		m_goingHome = false;
		emit moveForward(0);
		m_endPosition.reset();
	}

	if (m_datacounter % 5) {
		emit uiValuesChanged(x, y, fi);
	}

	m_datacounter++;
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
double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
double shift_theta(double theta)
{
    return 360.0 - theta;
}
/*
[180 0 0 -180]
*/
double shift_theta_robot(double theta)
{
    if(theta < 0){
        return theta + 360.0;
    }
    else{
        return theta;
    }
}
void MainWindow::updateLidarCircle(cv::Point center_of_object){
    m_objectDetected = center_of_object;
    m_detection_update_lidar = true;
}
void MainWindow::calculePositionOfObject(cv::Point center_of_object){
    //0 - 800 kamera pixel center
	double prob_angle;
	std::cout << "Center of object: " <<  center_of_object << std::endl;
	if(center_of_object.x <= 400)
		prob_angle = MAP(center_of_object.x, 0, 420, 30, 0);
	else
		prob_angle = MAP(center_of_object.x, 420, 840, 360, 330);
	std::cout << "Prob angle: " << prob_angle << std::endl;
	double dist = INT32_MAX;
	int k;
	double threshold = 1;
	double angle_norm;
	for(k = 0; k < m_copyOfLaserData.numberOfScans; k++){
		angle_norm = shift_theta( m_copyOfLaserData.Data[k].scanAngle);
		if(prob_angle-threshold < 0){
			if(angle_norm >= 360 + prob_angle - threshold || angle_norm <= prob_angle + threshold){
				if(dist > m_copyOfLaserData.Data[k].scanDistance && m_copyOfLaserData.Data[k].scanDistance != 0){
					dist = m_copyOfLaserData.Data[k].scanDistance/10.0;
					break;
				}
			}
		}else if(prob_angle+threshold > 360){
			if(angle_norm >= prob_angle - threshold || angle_norm <= prob_angle + threshold - 360){
				if(dist > m_copyOfLaserData.Data[k].scanDistance && m_copyOfLaserData.Data[k].scanDistance != 0){
					dist = m_copyOfLaserData.Data[k].scanDistance/10.0;
					break;
				}
			}
		}else{
			if(angle_norm >= prob_angle - threshold && angle_norm <= prob_angle + threshold){
				if(dist > m_copyOfLaserData.Data[k].scanDistance && m_copyOfLaserData.Data[k].scanDistance != 0){
					dist = m_copyOfLaserData.Data[k].scanDistance/10.0;
					break;
				}
			}
		}
	}
	std::cout << "distance: " << dist << std::endl;
	std::cout << "Scan angle: " << angle_norm << std::endl;
	std::cout << "k: " << k << std::endl;
	robot_x_find_object = m_x;
	robot_y_find_object = m_y;
	if(m_fi < 0)
		robot_fi_find_object = m_fi+2*M_PI;
	else
		robot_fi_find_object = m_fi;
	std::cout << "norm angle " << angle_norm << std::endl;
	std::cout << "robot fi " << shift_theta_robot(m_fi*180.0/M_PI) << std::endl;
	m_objectOnMap = cv::Point(dist * cos((shift_theta_robot(m_fi*180.0/M_PI) + angle_norm) * M_PI / 180.0), dist * sin((shift_theta_robot(m_fi*180.0/M_PI) + angle_norm)  * M_PI / 180.0));  //distance from robot
	m_draw_c = true;
	std::cout << "Object on map: " << m_objectOnMap << std::endl;
}
void MainWindow::drawImageData(QPainter &painter, QRect &rect, bool mini)
{
	if (m_robot == nullptr) {
		qDebug() << "Robot is not connected";
		return;
	}

	QImage image = QImage((uchar *)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step,
						  QImage::Format_RGB888); //kopirovanie cvmat do qimage
	parse_lidar_data(m_copyOfLaserData, m_distanceFromWall);
	calc_colisions_points(m_copyOfLaserData, &m_colisionDetected);

	if(start_pressed)
        emit haffTransform(frame[actIndex]);  
	image = image.scaled(rect.width(), rect.height(), Qt::KeepAspectRatio);
	painter.drawImage(rect, image.rgbSwapped());

	if (m_colisionDetected && !mini) {
		uint16_t width = rect.width();
		uint16_t height = rect.height();

		emit moveForward(0);
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
		qDebug() << "Robot is not connected";
		return;
	}

	emit positionResults(robotdata, m_fiCorrection);
}
