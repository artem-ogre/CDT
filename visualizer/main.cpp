/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */
#include "CDT.h"
#include "VerifyTopology.h"

#include <fstream>
#include <iostream>
#include <limits>

#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QDir>
#include <QHBoxLayout>
#include <QListWidget>
#include <QMessageBox>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QSpinBox>
#include <QTextStream>

typedef float CoordType;
typedef CDT::Triangulation<CoordType> Triangulation;
typedef CDT::V2d<CoordType> V2d;
typedef CDT::Vertex<CoordType> Vertex;
typedef CDT::Triangle Triangle;
typedef CDT::Box2d<CoordType> Box2d;
typedef CDT::Edge Edge;

class CDTWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CDTWidget(QWidget* parent = NULL)
        : QWidget(parent)
#ifdef CDT_USE_BOOST
        , m_cdt(CDT::FindingClosestPoint::BoostRTree)
#else
        , m_cdt(CDT::FindingClosestPoint::ClosestRandom, 10)
#endif
        , m_ptLimit(9999999)
        , m_edgeLimit(9999999)
        , m_isHidePoints(false)
        , m_isHideSuperTri(false)
        , m_isRemoveOuter(false)
        , m_isRemoveOuterAndHoles(false)
        , m_translation(0., 0.)
        , m_scale(1.0)
    {
        setAutoFillBackground(false);
    }

    QSize sizeHint() const
    {
        return QSize(9001, 9001); // over 9000!
    }

public slots:
    void buildCDT(QListWidgetItem* item)
    {
        QDir dir = QDir(QString(), tr("*.txt"));
        const QString fileName = dir.filePath(item->text());
        readData(fileName);
        initTransform();
        updateCDT();
    }

    void setPointsLimit(int limit)
    {
        m_ptLimit = static_cast<std::size_t>(limit);
        updateCDT();
    }

    void setEdgeLimit(int limit)
    {
        m_edgeLimit = static_cast<std::size_t>(limit);
        updateCDT();
    }

    void hidePoints(int isHidePoints)
    {
        m_isHidePoints = (isHidePoints != 0);
        update();
    }

    void hideSuperTriangle(int isHideSuperTri)
    {
        m_isHideSuperTri = (isHideSuperTri != 0);
        updateCDT();
    }

    void removeOuterTriangles(int isRemoveOuter)
    {
        m_isRemoveOuter = (isRemoveOuter != 0);
        updateCDT();
    }

    void removeOuterTrianglesAndHoles(int isRemoveOuterAndHoles)
    {
        m_isRemoveOuterAndHoles = (isRemoveOuterAndHoles != 0);
        updateCDT();
    }

    void prtScn()
    {
        QFile file("cdt_screenshot.png");
        file.open(QIODevice::WriteOnly);
        QPixmap pixmap(rect().size());
        pixmap.fill(Qt::transparent);
        paint_(&pixmap);
        pixmap.save(&file, "PNG");
    }

    void saveToOff()
    {
        std::ofstream fout("out.off");
        fout.precision(std::numeric_limits<CoordType>::digits10 + 1);
        if(!fout.is_open())
            throw std::runtime_error("Save can't open file for writing OFF");
        fout << "OFF\n";

        fout << m_cdt.vertices.size() << ' ' << m_cdt.triangles.size()
             << " 0\n";
        // Write vertices
        const Box2d box = Box2d::envelop(m_points);
        const CoordType stZ =
            -std::fmax(box.max.x - box.min.x, box.max.y - box.min.y);
        std::size_t counter = 0;
        typedef Triangulation::VertexVec::const_iterator VCit;
        for(VCit v = m_cdt.vertices.begin(); v != m_cdt.vertices.end(); ++v)
        {
            const CoordType z = !m_isRemoveOuterAndHoles && !m_isRemoveOuter &&
                                        !m_isHideSuperTri && counter < 3
                                    ? stZ
                                    : 0.0;
            fout << v->pos.x << ' ' << v->pos.y << ' ' << z << "\n";
            counter++;
        }
        // Write faces
        typedef CDT::TriangleVec::const_iterator TCit;
        for(TCit t = m_cdt.triangles.begin(); t != m_cdt.triangles.end(); ++t)
        {
            fout << "3 " << t->vertices[0] << ' ' << t->vertices[1] << ' '
                 << t->vertices[2] << "\n";
        }
        fout.close();
    }

private:
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
        m_points.clear();
        for(std::size_t i = 0; i < nPts; ++i)
        {
            CoordType x1, y1;
            inStream >> x1 >> y1;
            m_points.push_back(V2d::make(x1, y1));
        }
        m_edges.clear();
        for(std::size_t i = 0; i < nEdges; ++i)
        {
            CDT::VertInd v1, v2;
            inStream >> v1 >> v2;
            m_edges.push_back(Edge(v1, v2));
        }
        inStream.skipWhiteSpace();
    }

    void updateCDT()
    {
#ifdef CDT_USE_BOOST
        m_cdt = Triangulation(CDT::FindingClosestPoint::BoostRTree);
#else
        m_cdt = Triangulation(CDT::FindingClosestPoint::ClosestRandom, 10);
#endif
        if(!m_points.empty())
        {
            std::vector<V2d> pts =
                m_ptLimit < m_points.size()
                    ? std::vector<V2d>(&m_points[0], &m_points[m_ptLimit])
                    : m_points;
            const std::vector<std::size_t> mapping = CDT::RemoveDuplicates(pts);
            m_cdt.insertVertices(pts);
            if(m_ptLimit >= m_points.size() && !m_edges.empty())
            {
                std::vector<Edge> edges =
                    m_edgeLimit < m_edges.size()
                        ? std::vector<Edge>(&m_edges[0], &m_edges[m_edgeLimit])
                        : m_edges;
                CDT::RemapEdges(edges, mapping);
                m_cdt.insertEdges(edges);
            }
            if(m_isRemoveOuterAndHoles)
                m_cdt.eraseOuterTrianglesAndHoles();
            else if(m_isRemoveOuter)
                m_cdt.eraseOuterTriangles();
            else if(m_isHideSuperTri)
                m_cdt.eraseSuperTriangle();
        }
        if(!CDT::verifyTopology(m_cdt))
        {
            QMessageBox errBox;
            errBox.setText(QStringLiteral("Triangulation has wrong topology"));
            errBox.exec();
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent*)
    {
        paint_(this);
    }

private:
    QPointF sceneToScreen(const V2d& xy) const
    {
        const QPointF screenCenter(width() / 2.0, height() / 2.0);
        return QPointF(m_scale * xy.x, -m_scale * xy.y) + screenCenter +
               m_translation;
    }
    QPointF screenToScene(const QPointF& xy) const
    {
        const QPointF screenCenter(width() / 2.0, height() / 2.0);
        QPointF out = (xy - m_translation - screenCenter) / m_scale;
        out.setY(-out.y());
        return out;
    }
    double calculateScale(const int w, const int h) const
    {
        const V2d sceneSize = V2d::make(
            m_sceneBox.max.x - m_sceneBox.min.x,
            m_sceneBox.max.y - m_sceneBox.min.y);
        const double sceneRatio = sceneSize.x / sceneSize.y;
        const double screenRatio = static_cast<double>(w) / h;
        double scale =
            (sceneRatio > screenRatio) ? w / sceneSize.x : h / sceneSize.y;
        return scale * 0.95;
    }
    void initTransform()
    {
        m_sceneBox = Box2d::envelop(m_points);
        QPointF sceneCenter(
            (m_sceneBox.min.x + m_sceneBox.max.x) / CoordType(2),
            (m_sceneBox.min.y + m_sceneBox.max.y) / CoordType(2));
        m_scale = calculateScale(width(), height());
        m_translation =
            QPointF(-m_scale * sceneCenter.x(), m_scale * sceneCenter.y());
    }

    void paint_(QPaintDevice* pd)
    {
        QPainter p(pd);
        p.setRenderHints(QPainter::Antialiasing);

        p.setBrush(QBrush(Qt::white));
        if(m_cdt.vertices.empty())
            return;

        QPen pen;
        pen.setCapStyle(Qt::RoundCap);

        // Draw triangles
        pen.setWidthF(2.0);
        // outer triangles
        if(!m_isHideSuperTri && !m_isRemoveOuter && !m_isRemoveOuterAndHoles)
        {
            pen.setColor(QColor(220, 220, 220));
            p.setPen(pen);
            typedef CDT::TriangleVec::const_iterator TCit;
            for(TCit t = m_cdt.triangles.begin(); t != m_cdt.triangles.end();
                ++t)
            {
                if(t->vertices[0] > 2 && t->vertices[1] > 2 &&
                   t->vertices[2] > 2)
                    continue;
                const V2d& v1 = m_cdt.vertices[t->vertices[0]].pos;
                const V2d& v2 = m_cdt.vertices[t->vertices[1]].pos;
                const V2d& v3 = m_cdt.vertices[t->vertices[2]].pos;
                const QPointF pt1 = sceneToScreen(v1);
                const QPointF pt2 = sceneToScreen(v2);
                const QPointF pt3 = sceneToScreen(v3);
                p.drawLine(pt1, pt2);
                p.drawLine(pt2, pt3);
                p.drawLine(pt3, pt1);
            }
        }

        // actual triangles
        pen.setColor(QColor(150, 150, 150));
        p.setPen(pen);
        typedef CDT::TriangleVec::const_iterator TCit;
        for(TCit t = m_cdt.triangles.begin(); t != m_cdt.triangles.end(); ++t)
        {
            if(!m_isHideSuperTri && !m_isRemoveOuter &&
               !m_isRemoveOuterAndHoles)
            {
                if(t->vertices[0] < 3 || t->vertices[1] < 3 ||
                   t->vertices[2] < 3)
                {
                    continue;
                }
            }
            const V2d& v1 = m_cdt.vertices[t->vertices[0]].pos;
            const V2d& v2 = m_cdt.vertices[t->vertices[1]].pos;
            const V2d& v3 = m_cdt.vertices[t->vertices[2]].pos;
            const CDT::array<QPointF, 3> pts = {
                sceneToScreen(v1), sceneToScreen(v2), sceneToScreen(v3)};
            p.drawPolygon(pts.begin(), pts.size());
        }
        // constraint edges
        pen.setColor(QColor(50, 50, 50));
        p.setPen(pen);
        typedef CDT::EdgeUSet::const_iterator ECit;
        for(ECit e = m_cdt.fixedEdges.begin(); e != m_cdt.fixedEdges.end(); ++e)
        {
            const V2d& v1 = m_cdt.vertices[e->v1()].pos;
            const V2d& v2 = m_cdt.vertices[e->v2()].pos;
            p.drawLine(sceneToScreen(v1), sceneToScreen(v2));
        }

        if(m_isHidePoints)
            return;
        // draw points
        pen.setColor(QColor(3, 102, 214));
        pen.setWidthF(7.0);
        p.setPen(pen);
        for(std::size_t i = 0; i < m_cdt.vertices.size(); ++i)
        {
            p.drawPoint(sceneToScreen(m_cdt.vertices[i].pos));
        }
        // last added point
        if(m_ptLimit <= m_points.size())
        {
            pen.setColor(QColor(200, 50, 50));
            pen.setWidthF(9.0);
            p.setPen(pen);
            const Vertex& v = m_cdt.vertices.back();
            p.drawPoint(sceneToScreen(v.pos));
        }
    }

    void mousePressEvent(QMouseEvent* event)
    {
        m_prevMousePos = event->pos();
        qApp->setOverrideCursor(Qt::ClosedHandCursor);
        setMouseTracking(true);
    }
    void mouseMoveEvent(QMouseEvent* event)
    {
        m_translation += (event->pos() - m_prevMousePos);
        m_prevMousePos = event->pos();
        update();
    }
    void mouseReleaseEvent(QMouseEvent*)
    {
        qApp->restoreOverrideCursor();
        setMouseTracking(false);
    }
    void wheelEvent(QWheelEvent* event)
    {
        const double newScale =
            m_scale * std::max(0.3, (1. + event->delta() * 8e-4));
        if(m_scale == newScale)
        {
            return;
        }
        const QPointF cursor = event->posF();
        const QPointF scenePt = screenToScene(cursor);
        const QPointF screenCenter = QPointF(width(), height()) / 2.0;
        m_translation = cursor - newScale * QPointF(scenePt.x(), -scenePt.y()) -
                        screenCenter;
        m_scale = newScale;
        update();
    }
    void resizeEvent(QResizeEvent* e)
    {
        const double scaleRatio =
            calculateScale(width(), height()) /
            calculateScale(e->oldSize().width(), e->oldSize().height());
        m_scale *= scaleRatio;
        m_translation *= scaleRatio;
        update();
    }

private:
    Triangulation m_cdt;
    std::vector<V2d> m_points;
    std::vector<Edge> m_edges;
    std::size_t m_ptLimit;
    std::size_t m_edgeLimit;
    bool m_isHidePoints;
    bool m_isHideSuperTri;
    bool m_isRemoveOuter;
    bool m_isRemoveOuterAndHoles;

    QPointF m_prevMousePos;
    QPointF m_translation;
    double m_scale;
    Box2d m_sceneBox;
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = Q_NULLPTR)
        : QWidget(parent)
    {
        m_cdtWidget = new CDTWidget();
        m_cdtWidget->setMinimumSize(QSize(1, 1));
        // Right pane
        QListWidget* filesList = new QListWidget();
        connect(
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

        QSpinBox* edgesSpinbox = new QSpinBox;
        edgesSpinbox->setRange(0, 999999);
        connect(
            edgesSpinbox,
            SIGNAL(valueChanged(int)),
            m_cdtWidget,
            SLOT(setEdgeLimit(int)));
        edgesSpinbox->setValue(999999);

        QCheckBox* hidePoints = new QCheckBox(QStringLiteral("Hide points"));
        connect(
            hidePoints,
            SIGNAL(stateChanged(int)),
            m_cdtWidget,
            SLOT(hidePoints(int)));
        m_cdtWidget->hidePoints(0);
        hidePoints->setChecked(false);

        QCheckBox* hideSuperTri =
            new QCheckBox(QStringLiteral("Hide super-triangle"));
        connect(
            hideSuperTri,
            SIGNAL(stateChanged(int)),
            m_cdtWidget,
            SLOT(hideSuperTriangle(int)));
        m_cdtWidget->hideSuperTriangle(0);
        hideSuperTri->setChecked(false);

        QCheckBox* removeOuter =
            new QCheckBox(QStringLiteral("Remove outer triangles"));
        connect(
            removeOuter,
            SIGNAL(stateChanged(int)),
            m_cdtWidget,
            SLOT(removeOuterTriangles(int)));
        m_cdtWidget->removeOuterTriangles(0);
        removeOuter->setChecked(false);

        QCheckBox* removeOuterHoles = new QCheckBox(
            QStringLiteral("Remove outer triangles, auto-detect holes"));
        connect(
            removeOuterHoles,
            SIGNAL(stateChanged(int)),
            m_cdtWidget,
            SLOT(removeOuterTrianglesAndHoles(int)));
        m_cdtWidget->removeOuterTrianglesAndHoles(0);
        removeOuterHoles->setChecked(false);

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
        rightLayout->addWidget(edgesSpinbox, cntr++, 0);
        rightLayout->addWidget(hidePoints, cntr++, 0);
        rightLayout->addWidget(removeOuter, cntr++, 0);
        rightLayout->addWidget(removeOuterHoles, cntr++, 0);
        rightLayout->addWidget(hideSuperTri, cntr++, 0);
        rightLayout->addWidget(screenshotBtn, cntr++, 0);
        rightLayout->addWidget(saveBtn, cntr++, 0);

        // Center
        QHBoxLayout* centralLayout = new QHBoxLayout;
        centralLayout->addWidget(m_cdtWidget);
        centralLayout->addLayout(rightLayout);
        setLayout(centralLayout);

        setWindowTitle(tr("CDT Visualizer"));

        // Read files list
        QDir dir = QDir(QString(), tr("*.txt"));
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
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return QApplication::exec();
}

#include "main.moc"
