#include "CDT.h"

#include <QApplication>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTextStream>
#include <QDir>
#include <QPainter>
#include <QColor>

typedef double CoordType;
typedef CDT::Triangulation<CoordType> Triangulation;
typedef CDT::V2d<CoordType> V2d;
typedef CDT::Vertex<CoordType> Vertex;
typedef CDT::Triangle Triangle;
typedef CDT::Box2d<CoordType> Box2d;
typedef CDT::Index Index;
const int fixedSize = 600;

class CDTWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CDTWidget(QWidget* parent = NULL)
        : QWidget(parent)
    {
        startTimer(40);
    }

    QSize sizeHint() const
    {
        return QSize(fixedSize, fixedSize);
    }

public slots:
    void buildCDT(QListWidgetItem* item)
    {
        QDir dir = QDir(QDir::currentPath(), tr("*.txt"));
        dir.setPath(QStringLiteral("../test files"));
        const QString fileName = dir.filePath(item->text());
        readData(fileName);
        updateCDT();
    }

    void setPointsLimit(int limit)
    {
        m_ptLimit = static_cast<std::size_t>(limit);
        updateCDT();
    }

    void prtScn()
    {}

    void saveToOff()
    {}

private:
    void timerEvent(QTimerEvent* /*e*/)
    {
        update();
    }

    void readData(const QString& file)
    {
        QFile data(file);
        if(!data.open(QFile::ReadOnly))
        {
            QMessageBox::warning(
                this, tr("CDT"), tr("Could not open file ") + file);
        }
        QTextStream inStream(&data);
        std::size_t nPts, nEdges;
        inStream >> nPts >> nEdges;
        for(std::size_t i = 0; i < nPts; ++i)
        {
            CoordType x1, y1;
            inStream >> x1 >> y1;
            m_points.push_back(V2d::make(x1, y1));
        }
        for(std::size_t i = 0; i < nEdges; ++i)
        {
            CDT::VertInd v1, v2;
            inStream >> v1 >> v2;
            m_edges.push_back(std::make_pair(v1, v2));
        }
        inStream.skipWhiteSpace();
    }

    void updateCDT()
    {
        m_cdt = Triangulation();
        if(m_ptLimit < m_points.size())
        {
            std::vector<V2d> pts(&m_points[0], &m_points[m_ptLimit]);
            m_cdt.insertVertices(pts);
        }
        else
        {
            m_cdt.insertVertices(m_points);
            //m_cdt.insertEdges(m_edges);
        }
    }

protected:
    void paintEvent(QPaintEvent*)
    {
        if(m_cdt.vertices.empty())
            return;

        QPainter p(this);
        p.setRenderHints(
            QPainter::Antialiasing | QPainter::HighQualityAntialiasing);

        p.translate(fixedSize / 2.0, fixedSize / 2.0);
        p.scale(1, -1);

        const Box2d box = calculateBox(m_points);
        const V2d c = {(box.min.x + box.max.x) / 2.0,
                            (box.min.y + box.max.y) / 2.0};
        const double scale = 0.8 * fixedSize / (box.max.x - box.min.x);

        QPen pen;
        pen.setCapStyle(Qt::RoundCap);

        // Draw triangles
        pen.setWidthF(2.0);
        // outer triangles
        pen.setColor(QColor(220, 220, 220));
        p.setPen(pen);
        BOOST_FOREACH(const Triangle& t, m_cdt.triangles)
        {
            if(t.vertices[0] > 2 && t.vertices[1] > 2 && t.vertices[2] > 2)
                continue;
            const V2d& v1 = m_cdt.vertices[t.vertices[0]].pos;
            const V2d& v2 = m_cdt.vertices[t.vertices[1]].pos;
            const V2d& v3 = m_cdt.vertices[t.vertices[2]].pos;
            const QPointF pt1(scale * (v1.x - c.x), scale * (v1.y - c.y));
            const QPointF pt2(scale * (v2.x - c.x), scale * (v2.y - c.y));
            const QPointF pt3(scale * (v3.x - c.x), scale * (v3.y - c.y));
            p.drawLine(pt1, pt2);
            p.drawLine(pt2, pt3);
            p.drawLine(pt3, pt1);
        }
        // actual triangles
        pen.setColor(QColor(150, 150, 150));
        p.setPen(pen);
        BOOST_FOREACH(const Triangle& t, m_cdt.triangles)
        {
            if(t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3)
                continue;
            const V2d& v1 = m_cdt.vertices[t.vertices[0]].pos;
            const V2d& v2 = m_cdt.vertices[t.vertices[1]].pos;
            const V2d& v3 = m_cdt.vertices[t.vertices[2]].pos;
            const QPointF pt1(scale * (v1.x - c.x), scale * (v1.y - c.y));
            const QPointF pt2(scale * (v2.x - c.x), scale * (v2.y - c.y));
            const QPointF pt3(scale * (v3.x - c.x), scale * (v3.y - c.y));
            p.drawLine(pt1, pt2);
            p.drawLine(pt2, pt3);
            p.drawLine(pt3, pt1);
        }

        // Draw points
        pen.setColor(QColor(50, 50, 200));
        pen.setWidthF(5.0);
        p.setPen(pen);
        for(std::size_t i = 3; i < m_cdt.vertices.size(); ++i)
        {
            const Vertex& v = m_cdt.vertices[i];
            const QPointF pt(scale * (v.pos.x - c.x), scale * (v.pos.y - c.y));
            p.drawPoint(pt);
        }
    }

private:
    Triangulation m_cdt;
    std::size_t m_ptLimit;
    std::vector<V2d> m_points;
    std::vector<Triangulation::Edge> m_edges;
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = Q_NULLPTR)
        : QWidget(parent)
    {
        m_cdtWidget = new CDTWidget();
        m_cdtWidget->setFixedSize(QSize(fixedSize,fixedSize));
        // Right pane
        QListWidget* filesList = new QListWidget();
        filesList->connect(
            filesList,
            SIGNAL(itemDoubleClicked(QListWidgetItem*)),
            m_cdtWidget,
            SLOT(buildCDT(QListWidgetItem*)));

        QSpinBox* ptsSpinbox = new QSpinBox;
        ptsSpinbox->setRange(0, 999999);
        connect(
            ptsSpinbox,
            SIGNAL(valueChanged(int)),
            m_cdtWidget,
            SLOT(setPointsLimit(int)));
        ptsSpinbox->setValue(999999);

        QPushButton* screenshotBtn = new QPushButton(tr("Make Screenshot"));
        connect(screenshotBtn, SIGNAL(clicked()), m_cdtWidget, SLOT(prtScn()));
        screenshotBtn->setMinimumHeight(50);

        QPushButton* saveBtn = new QPushButton(tr("Save to .OFF"));
        connect(saveBtn, SIGNAL(clicked()), m_cdtWidget, SLOT(saveToOff()));
        saveBtn->setMinimumHeight(50);

        QGridLayout* rightLayout = new QGridLayout;
        int cntr = 0;
        rightLayout->addWidget(filesList, cntr++, 0);
        rightLayout->addWidget(ptsSpinbox, cntr++, 0);
        rightLayout->addWidget(screenshotBtn, cntr++, 0);
        rightLayout->addWidget(saveBtn, cntr++, 0);

        // Center
        QHBoxLayout* centralLayout = new QHBoxLayout;
        centralLayout->addWidget(m_cdtWidget);
        centralLayout->addLayout(rightLayout);
        setLayout(centralLayout);

        setWindowTitle(tr("CDT Visualizer"));
        layout()->setSizeConstraint(QLayout::SetFixedSize);

        // Read files list
        QDir dir = QDir(QDir::currentPath(), tr("*.txt"));
        dir.setPath(QStringLiteral("../test files"));
        QFileInfoList list = dir.entryInfoList();
        filesList->clear();
        QFileInfoList::iterator it;
        for(it = list.begin(); it != list.end(); ++it)
            filesList->addItem(it->fileName());
        filesList->setCurrentRow(0);
    }

private:
    CDTWidget* m_cdtWidget;
};

int main(int argc, char* argv[])
{
    Triangulation cdt;
    {
        std::vector<V2d> vertices;
        vertices.push_back(V2d::make(0, 0));
        vertices.push_back(V2d::make(1, 0));
        vertices.push_back(V2d::make(2, 1));
        vertices.push_back(V2d::make(1, 1));
        cdt.insertVertices(vertices);
        const std::vector<Triangulation::Edge> edges(1, std::make_pair(0, 2));
        cdt.insertEdges(edges);
    }
    { // Check if on triangle edge case is detected
        const V2d a = {0, 0};
        const V2d b = {10, 0};
        const V2d c = {10, 10};
        const V2d pt = {5, 5};
        CDT::PtTriLocation::Enum tmp = CDT::locatePointTriangle(pt, a, b, c);
    }
    cdt = Triangulation();
    {
        std::vector<V2d> vertices;
        vertices.push_back(V2d::make(0, 0));
        vertices.push_back(V2d::make(2, 0));
        vertices.push_back(V2d::make(2, 2));
        vertices.push_back(V2d::make(1, 1));
        vertices.push_back(V2d::make(3.0 / 2.0, 1.0 / 2.0));
        cdt.insertVertices(vertices);
    }
    cdt = Triangulation();
    {
        std::vector<V2d> vertices;
        vertices.push_back(V2d::make(0, 0));
        vertices.push_back(V2d::make(2, 0));
        vertices.push_back(V2d::make(1, 3));
        vertices.push_back(V2d::make(2, 2));
        cdt.insertVertices(vertices);
        std::vector<Triangulation::Edge> edges = {std::make_pair(1, 2)};
        cdt.insertEdges(edges);
    }

    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

#include "main.moc"
