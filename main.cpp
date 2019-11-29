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
#include <QFileDialog>
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
typedef CDT::Index Index;
typedef CDT::Edge Edge;

class CDTWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CDTWidget(QWidget* parent = NULL)
        : QWidget(parent)
#ifndef CDT_DONT_USE_BOOST_RTREE
        , m_cdt(CDT::FindingClosestPoint::BoostRTree)
#else
        , m_cdt(CDT::FindingClosestPoint::ClosestRandom, 10)
#endif
        , m_ptLimit(9999999)
        , m_edgeLimit(9999999)
        , m_isHidePoints(false)
        , m_isHideSuperTri(false)
        , m_isRemoveOuter(false)
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
        QDir dir = QDir(QDir::currentPath(), tr("*.txt"));
        dir.setPath(QStringLiteral("test files"));
        const QString fileName = dir.filePath(item->text());
        readData(fileName);
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

    void prtScn()
    {}

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
            const CoordType z =
                !m_isRemoveOuter && !m_isHideSuperTri && counter < 3 ? stZ
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
#ifndef CDT_DONT_USE_BOOST_RTREE
        m_cdt = Triangulation(CDT::FindingClosestPoint::BoostRTree);
#else
        m_cdt = Triangulation(CDT::FindingClosestPoint::ClosestRandom, 10);
#endif
        if(!m_points.empty())
        {
            const std::vector<V2d> pts =
                m_ptLimit < m_points.size()
                    ? std::vector<V2d>(&m_points[0], &m_points[m_ptLimit])
                    : m_points;
            m_cdt.insertVertices(pts);
            if(m_ptLimit >= m_points.size() && !m_edges.empty())
            {
                const std::vector<Edge> edges =
                    m_edgeLimit < m_edges.size()
                        ? std::vector<Edge>(&m_edges[0], &m_edges[m_edgeLimit])
                        : m_edges;
                m_cdt.insertEdges(edges);
            }
            if(m_isRemoveOuter)
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
        QPainter p(this);
        if(m_cdt.vertices.empty())
            return;

        const CoordType fixedSize(std::min(size().width(), size().height()));
        p.setRenderHints(QPainter::Antialiasing);
        p.translate(fixedSize / 2.0, fixedSize / 2.0);
        p.scale(1, -1);

        const Box2d box = Box2d::envelop(m_points);
        const V2d c = {(box.min.x + box.max.x) / CoordType(2),
                       (box.min.y + box.max.y) / CoordType(2)};
        const double scale =
            0.8 * fixedSize /
            (std::fmax(box.max.x - box.min.x, box.max.y - box.min.y));

        QPen pen;
        pen.setCapStyle(Qt::RoundCap);

        // Draw triangles
        pen.setWidthF(2.0);
        // outer triangles
        if(!m_isHideSuperTri && !m_isRemoveOuter)
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
                const QPointF pt1(scale * (v1.x - c.x), scale * (v1.y - c.y));
                const QPointF pt2(scale * (v2.x - c.x), scale * (v2.y - c.y));
                const QPointF pt3(scale * (v3.x - c.x), scale * (v3.y - c.y));
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
            if(!m_isHideSuperTri && !m_isRemoveOuter)
                if(t->vertices[0] < 3 || t->vertices[1] < 3 || t->vertices[2] < 3)
                    continue;
            const V2d& v1 = m_cdt.vertices[t->vertices[0]].pos;
            const V2d& v2 = m_cdt.vertices[t->vertices[1]].pos;
            const V2d& v3 = m_cdt.vertices[t->vertices[2]].pos;
            const QPointF pt1(scale * (v1.x - c.x), scale * (v1.y - c.y));
            const QPointF pt2(scale * (v2.x - c.x), scale * (v2.y - c.y));
            const QPointF pt3(scale * (v3.x - c.x), scale * (v3.y - c.y));
            p.drawLine(pt1, pt2);
            p.drawLine(pt2, pt3);
            p.drawLine(pt3, pt1);
        }
        // constraint edges
        pen.setColor(QColor(50, 50, 50));
        p.setPen(pen);
        typedef CDT::EdgeUSet::const_iterator ECit;
        for(ECit e = m_cdt.fixedEdges.begin(); e != m_cdt.fixedEdges.end(); ++e)
        {
            const V2d& v1 = m_cdt.vertices[e->v1()].pos;
            const V2d& v2 = m_cdt.vertices[e->v2()].pos;
            const QPointF pt1(scale * (v1.x - c.x), scale * (v1.y - c.y));
            const QPointF pt2(scale * (v2.x - c.x), scale * (v2.y - c.y));
            p.drawLine(pt1, pt2);
        }

        if(m_isHidePoints)
            return;
        // draw points
        pen.setColor(QColor(50, 50, 200));
        pen.setWidthF(7.0);
        p.setPen(pen);
        for(std::size_t i = 0; i < m_cdt.vertices.size(); ++i)
        {
            const Vertex& v = m_cdt.vertices[i];
            const QPointF pt(scale * (v.pos.x - c.x), scale * (v.pos.y - c.y));
            p.drawPoint(pt);
        }
        // last added point
        if(m_ptLimit <= m_points.size())
        {
            pen.setColor(QColor(200, 50, 50));
            pen.setWidthF(9.0);
            p.setPen(pen);
            const Vertex& v = m_cdt.vertices.back();
            const QPointF pt(scale * (v.pos.x - c.x), scale * (v.pos.y - c.y));
            p.drawPoint(pt);
        }
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
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = Q_NULLPTR)
        : QWidget(parent)
    {
        m_cdtWidget = new CDTWidget();
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
        QDir dir = QDir(QDir::currentPath(), tr("*.txt"));
        dir.setPath(QStringLiteral("test files"));
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
