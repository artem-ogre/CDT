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
#include <QComboBox>
#include <QDir>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QSpinBox>
#include <QTextStream>

typedef double CoordType;
typedef CDT::Triangulation<CoordType> Triangulation;
typedef CDT::V2d<CoordType> V2d;
typedef CDT::Triangle Triangle;
typedef CDT::Box2d<CoordType> Box2d;
typedef CDT::Edge Edge;

enum class TriangulationType
{
    ConstraintDelaunay,
    ConformingDelaunay,
};

enum class FinalizeTriangulation
{
    DontFinalize,
    EraseSuperTriangle,
    EraseOuterTriangles,
    EraseOuterTrianglesAndHoles,
};

class CDTWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CDTWidget(QWidget* parent = NULL)
        : QWidget(parent)
        , m_ptLimit(9999999)
        , m_edgeLimit(9999999)
        , m_vertexInsertionOrder(CDT::VertexInsertionOrder::Enum::Auto)
        , m_intersectingEdgesStrategy(
              CDT::IntersectingConstraintEdges::Enum::TryResolve)
        , m_minDistToConstraintEdge(1e-6)
        , m_triangulationType(TriangulationType::ConstraintDelaunay)
        , m_finalizeType(FinalizeTriangulation::DontFinalize)
        , m_fixDuplicates(true)
        , m_isHidePoints(false)
        , m_isDisplayIndices(false)
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

    void setTriangulationType(int index)
    {
        switch(index)
        {
        case 0:
            m_triangulationType = TriangulationType::ConstraintDelaunay;
            break;
        case 1:
            m_triangulationType = TriangulationType::ConformingDelaunay;
            break;
        }
        updateCDT();
    }

    void setMinDistToConstraintEdge(double tolerance)
    {
        m_minDistToConstraintEdge = tolerance;
        updateCDT();
    }

    void setVertexInsertionOrder(int index)
    {
        switch(index)
        {
        case 0:
            m_vertexInsertionOrder = CDT::VertexInsertionOrder::Enum::Auto;
            break;
        case 1:
            m_vertexInsertionOrder =
                CDT::VertexInsertionOrder::Enum::AsProvided;
            break;
        }
        updateCDT();
    }

    void setIntersectingEdgesStrategy(int index)
    {
        switch(index)
        {
        case 0:
            m_intersectingEdgesStrategy =
                CDT::IntersectingConstraintEdges::Enum::NotAllowed;
            break;
        case 1:
            m_intersectingEdgesStrategy =
                CDT::IntersectingConstraintEdges::Enum::TryResolve;
            break;
        case 2:
            m_intersectingEdgesStrategy =
                CDT::IntersectingConstraintEdges::Enum::DontCheck;
            break;
        }
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

    void setFinalizeType(int index)
    {
        switch(index)
        {
        case 0:
            m_finalizeType = FinalizeTriangulation::DontFinalize;
            break;
        case 1:
            m_finalizeType = FinalizeTriangulation::EraseSuperTriangle;
            break;
        case 2:
            m_finalizeType = FinalizeTriangulation::EraseOuterTriangles;
            break;
        case 3:
            m_finalizeType = FinalizeTriangulation::EraseOuterTrianglesAndHoles;
            break;
        }
        updateCDT();
    }

    void setFixDuplicates(int index)
    {
        m_fixDuplicates = (index == 0);
        updateCDT();
    }

    void displayIndices(int isDisplayIndices)
    {
        m_isDisplayIndices = (isDisplayIndices != 0);
        update();
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
        const Box2d box = envelopBox(m_points);
        const CoordType stZ =
            -std::fmax(box.max.x - box.min.x, box.max.y - box.min.y);
        std::size_t counter = 0;
        typedef Triangulation::V2dVec::const_iterator VCit;
        for(VCit v = m_cdt.vertices.begin(); v != m_cdt.vertices.end(); ++v)
        {
            const CoordType z =
                m_finalizeType == FinalizeTriangulation::DontFinalize &&
                        counter < 3
                    ? stZ
                    : 0.0;
            fout << v->x << ' ' << v->y << ' ' << z << "\n";
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
    void readData(const QString& fileName)
    {
        QFile file(fileName);
        if(!file.open(QFile::ReadOnly))
        {
            QMessageBox::warning(
                this, tr("CDT"), tr("Could not open file ") + fileName);
        }
        QTextStream inStream(&file);
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
        m_cdt = Triangulation(
            m_vertexInsertionOrder,
            m_intersectingEdgesStrategy,
            m_minDistToConstraintEdge);
        if(!m_points.empty())
        {
            std::vector<V2d> pts =
                m_ptLimit < m_points.size()
                    ? std::vector<V2d>(&m_points[0], &m_points[m_ptLimit])
                    : m_points;

            CDT::DuplicatesInfo dupeInfo;
            if(m_fixDuplicates)
            {
                dupeInfo = CDT::RemoveDuplicates(pts);
                if(!dupeInfo.duplicates.empty())
                {
                    QMessageBox errBox;
                    errBox.setText("Duplicate vertices were found and fixed");
                    errBox.exec();
                }
            }

            try
            {
                m_cdt.insertVertices(pts);
            }
            catch(const CDT::Error& e)
            {
                QMessageBox errBox;
                errBox.setText(e.what());
                errBox.exec();
                return;
            }
            if(m_ptLimit >= m_points.size() && !m_edges.empty())
            {
                std::vector<Edge> edges =
                    m_edgeLimit < m_edges.size()
                        ? std::vector<Edge>(&m_edges[0], &m_edges[m_edgeLimit])
                        : m_edges;
                if(m_fixDuplicates)
                {
                    CDT::RemapEdges(edges, dupeInfo.mapping);
                }
                try
                {
                    switch(m_triangulationType)
                    {
                    case TriangulationType::ConstraintDelaunay:
                        m_cdt.insertEdges(edges);
                        break;
                    case TriangulationType::ConformingDelaunay:
                        m_cdt.conformToEdges(edges);
                        break;
                    }
                }
                catch(const CDT::Error& e)
                {
                    QMessageBox errBox;
                    errBox.setText(e.what());
                    errBox.exec();
                    return;
                }
            }
            switch(m_finalizeType)
            {
            case FinalizeTriangulation::DontFinalize:
                break;
            case FinalizeTriangulation::EraseSuperTriangle:
                m_cdt.eraseSuperTriangle();
                break;
            case FinalizeTriangulation::EraseOuterTriangles:
                m_cdt.eraseOuterTriangles();
                break;
            case FinalizeTriangulation::EraseOuterTrianglesAndHoles:
                m_cdt.eraseOuterTrianglesAndHoles();
                break;
            }
            const CDT::unordered_map<Edge, CDT::EdgeVec> tmp =
                CDT::EdgeToPiecesMapping(m_cdt.pieceToOriginals);
            const CDT::unordered_map<Edge, std::vector<CDT::VertInd> >
                edgeToSplitVerts = EdgeToSplitVertices(tmp, m_cdt.vertices);
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
        m_sceneBox = envelopBox(m_points);
        QPointF sceneCenter(
            (m_sceneBox.min.x + m_sceneBox.max.x) / CoordType(2),
            (m_sceneBox.min.y + m_sceneBox.max.y) / CoordType(2));
        m_scale = calculateScale(width(), height());
        m_translation =
            QPointF(-m_scale * sceneCenter.x(), m_scale * sceneCenter.y());
    }

    void paint_(QPaintDevice* pd)
    {
        const QColor highlightColor(100, 100, 0);
        const QColor outerTrisColor(220, 220, 220);
        const QColor trianglesColor(150, 150, 150);
        const QColor fixedEdgeColor(50, 50, 50);
        const QColor pointColor(3, 102, 214);
        const QColor pointLabelColor(150, 0, 150);
        const QColor triangleLabelColor(0, 150, 150);

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
        if(m_finalizeType == FinalizeTriangulation::DontFinalize)
        {
            pen.setColor(outerTrisColor);
            p.setPen(pen);
            typedef CDT::TriangleVec::const_iterator TCit;
            for(TCit t = m_cdt.triangles.begin(); t != m_cdt.triangles.end();
                ++t)
            {
                if(t->vertices[0] > 2 && t->vertices[1] > 2 &&
                   t->vertices[2] > 2)
                    continue;
                const V2d& v1 = m_cdt.vertices[t->vertices[0]];
                const V2d& v2 = m_cdt.vertices[t->vertices[1]];
                const V2d& v3 = m_cdt.vertices[t->vertices[2]];
                const QPointF pt1 = sceneToScreen(v1);
                const QPointF pt2 = sceneToScreen(v2);
                const QPointF pt3 = sceneToScreen(v3);
                p.drawLine(pt1, pt2);
                p.drawLine(pt2, pt3);
                p.drawLine(pt3, pt1);
            }
        }

        // actual triangles
        pen.setColor(trianglesColor);
        p.setPen(pen);
        typedef CDT::TriangleVec::const_iterator TCit;
        int iT = 0;
        for(TCit t = m_cdt.triangles.begin(); t != m_cdt.triangles.end();
            ++t, ++iT)
        {
            if(m_finalizeType == FinalizeTriangulation::DontFinalize)
            {
                if(t->vertices[0] < 3 || t->vertices[1] < 3 ||
                   t->vertices[2] < 3)
                {
                    continue;
                }
            }
            const V2d& v1 = m_cdt.vertices[t->vertices[0]];
            const V2d& v2 = m_cdt.vertices[t->vertices[1]];
            const V2d& v3 = m_cdt.vertices[t->vertices[2]];
            const CDT::array<QPointF, 3> pts = {
                sceneToScreen(v1), sceneToScreen(v2), sceneToScreen(v3)};
            const QPointF c(
                (pts[0].x() + pts[1].x() + pts[2].x()) / 3.f,
                (pts[0].y() + pts[1].y() + pts[2].y()) / 3.f);
            p.drawPolygon(pts.data(), pts.size());
        }
        if(m_isDisplayIndices)
        {
            pen.setColor(triangleLabelColor);
            p.setPen(pen);
            iT = 0;
            for(TCit t = m_cdt.triangles.begin(); t != m_cdt.triangles.end();
                ++t, ++iT)
            {
                const V2d& v1 = m_cdt.vertices[t->vertices[0]];
                const V2d& v2 = m_cdt.vertices[t->vertices[1]];
                const V2d& v3 = m_cdt.vertices[t->vertices[2]];
                const CDT::array<QPointF, 3> pts = {
                    sceneToScreen(v1), sceneToScreen(v2), sceneToScreen(v3)};
                const QPointF c(
                    (pts[0].x() + pts[1].x() + pts[2].x()) / 3.f,
                    (pts[0].y() + pts[1].y() + pts[2].y()) / 3.f);
                p.drawText(c, QString::number(iT));
            }
        }
        // constraint edges
        pen.setColor(fixedEdgeColor);
        p.setPen(pen);
        typedef CDT::EdgeUSet::const_iterator ECit;
        for(ECit e = m_cdt.fixedEdges.begin(); e != m_cdt.fixedEdges.end(); ++e)
        {
            const V2d& v1 = m_cdt.vertices[e->v1()];
            const V2d& v2 = m_cdt.vertices[e->v2()];
            p.drawLine(sceneToScreen(v1), sceneToScreen(v2));
        }
        // last added edge
        if(m_edgeLimit && m_edgeLimit <= m_edges.size())
        {
            pen.setColor(highlightColor);
            pen.setWidthF(4.0);
            p.setPen(pen);
            p.drawLine(
                sceneToScreen(m_points[m_edges[m_edgeLimit - 1].v1()]),
                sceneToScreen(m_points[m_edges[m_edgeLimit - 1].v2()]));
        }

        if(m_isHidePoints)
            return;
        // draw points
        pen.setColor(pointColor);
        pen.setWidthF(7.0);
        p.setPen(pen);
        for(std::size_t i = 0; i < m_cdt.vertices.size(); ++i)
        {
            const QPointF pos = sceneToScreen(m_cdt.vertices[i]);
            p.drawPoint(pos);
        }
        if(m_isDisplayIndices)
        {
            pen.setColor(pointLabelColor);
            p.setPen(pen);
            for(std::size_t i = 0; i < m_cdt.vertices.size(); ++i)
            {
                const QPointF pos = sceneToScreen(m_cdt.vertices[i]);
                p.drawText(pos, QString::number(i));
            }
        }
        // last added point
        if(m_ptLimit && m_ptLimit <= m_points.size())
        {
            pen.setColor(highlightColor);
            pen.setWidthF(9.0);
            p.setPen(pen);
            p.drawPoint(sceneToScreen(m_points[m_ptLimit - 1]));
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
            m_scale * std::max(0.3, (1. + event->angleDelta().y() * 8e-4));
        if(m_scale == newScale)
        {
            return;
        }
        const QPointF cursor = event->position();
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
    CDT::VertexInsertionOrder::Enum m_vertexInsertionOrder;
    CDT::IntersectingConstraintEdges::Enum m_intersectingEdgesStrategy;
    CoordType m_minDistToConstraintEdge;
    TriangulationType m_triangulationType;
    FinalizeTriangulation m_finalizeType;
    bool m_fixDuplicates;
    bool m_isHidePoints;
    bool m_isDisplayIndices;

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
        QPushButton* refreshFiles = new QPushButton(tr("Refresh files list"));
        connect(refreshFiles, SIGNAL(clicked()), this, SLOT(updateFilesList()));

        m_filesList = new QListWidget();
        connect(
            m_filesList,
            SIGNAL(itemDoubleClicked(QListWidgetItem*)),
            m_cdtWidget,
            SLOT(buildCDT(QListWidgetItem*)));

        QComboBox* vOrder = new QComboBox;
        vOrder->addItem("Auto");
        vOrder->addItem("AsProvided");
        connect(
            vOrder,
            SIGNAL(currentIndexChanged(int)),
            m_cdtWidget,
            SLOT(setVertexInsertionOrder(int)));

        QComboBox* isecStrategy = new QComboBox;
        isecStrategy->addItem("NotAllowed");
        isecStrategy->addItem("TryResolve");
        isecStrategy->addItem("DontCheck");
        isecStrategy->setCurrentIndex(1);
        connect(
            isecStrategy,
            SIGNAL(currentIndexChanged(int)),
            m_cdtWidget,
            SLOT(setIntersectingEdgesStrategy(int)));

        QDoubleSpinBox* distTolerance = new QDoubleSpinBox;
        distTolerance->setDecimals(6);
        distTolerance->setRange(0.0, 0.1);
        distTolerance->setSingleStep(1e-6);
        distTolerance->setValue(1e-6);
        connect(
            distTolerance,
            SIGNAL(valueChanged(double)),
            m_cdtWidget,
            SLOT(setMinDistToConstraintEdge(double)));

        QComboBox* triType = new QComboBox;
        triType->addItem("constraint Delaunay triangulation");
        triType->addItem("conforming Delaunay triangulation");
        connect(
            triType,
            SIGNAL(currentIndexChanged(int)),
            m_cdtWidget,
            SLOT(setTriangulationType(int)));

        QComboBox* finalizeWith = new QComboBox;
        finalizeWith->addItem("don't finalize");
        finalizeWith->addItem("eraseSuperTriangle");
        finalizeWith->addItem("eraseOuterTriangles");
        finalizeWith->addItem("eraseOuterTrianglesAndHoles");
        connect(
            finalizeWith,
            SIGNAL(currentIndexChanged(int)),
            m_cdtWidget,
            SLOT(setFinalizeType(int)));

        QComboBox* fixDupes = new QComboBox;
        fixDupes->addItem("fix duplicated vertices and remap edges");
        fixDupes->addItem("don't fix duplicated vertices");
        connect(
            fixDupes,
            SIGNAL(currentIndexChanged(int)),
            m_cdtWidget,
            SLOT(setFixDuplicates(int)));

        QFormLayout* triangulationConfig = new QFormLayout;
        triangulationConfig->addRow(new QLabel("vertexInsertionOrder"), vOrder);
        triangulationConfig->addRow(
            new QLabel("intersectingEdgesStrategy"), isecStrategy);
        triangulationConfig->addRow(
            new QLabel("minDistToConstraintEdge"), distTolerance);
        triangulationConfig->addRow(triType);
        triangulationConfig->addRow(finalizeWith);
        triangulationConfig->addRow(fixDupes);
        QGroupBox* triOptGroup = new QGroupBox("Triangulation");
        triOptGroup->setLayout(triangulationConfig);

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

        QFormLayout* limitsLayout = new QFormLayout;
        limitsLayout->addRow(new QLabel(tr("Points")), ptsSpinbox);
        limitsLayout->addRow(new QLabel(tr("Edges")), edgesSpinbox);
        QGroupBox* limitsGroup = new QGroupBox("Limits");
        limitsGroup->setLayout(limitsLayout);

        QCheckBox* displayIndices =
            new QCheckBox(QStringLiteral("Display point/triangle indices"));
        connect(
            displayIndices,
            SIGNAL(stateChanged(int)),
            m_cdtWidget,
            SLOT(displayIndices(int)));
        m_cdtWidget->displayIndices(0);
        displayIndices->setChecked(false);

        QCheckBox* hidePoints = new QCheckBox(QStringLiteral("Hide points"));
        connect(
            hidePoints,
            SIGNAL(stateChanged(int)),
            m_cdtWidget,
            SLOT(hidePoints(int)));
        m_cdtWidget->hidePoints(0);
        hidePoints->setChecked(false);

        QFormLayout* visOptions = new QFormLayout;
        visOptions->addRow(displayIndices);
        visOptions->addRow(hidePoints);
        QGroupBox* visOptionsGroup = new QGroupBox("Visualization");
        visOptionsGroup->setLayout(visOptions);

        QPushButton* screenshotBtn = new QPushButton(tr("Make Screenshot"));
        connect(screenshotBtn, SIGNAL(clicked()), m_cdtWidget, SLOT(prtScn()));

        QPushButton* saveBtn = new QPushButton(tr("Save to .OFF"));
        connect(saveBtn, SIGNAL(clicked()), m_cdtWidget, SLOT(saveToOff()));

        QGridLayout* rightLayout = new QGridLayout;
        int cntr = 0;
        rightLayout->addWidget(refreshFiles, cntr++, 0);
        rightLayout->addWidget(m_filesList, cntr++, 0);
        rightLayout->addWidget(triOptGroup, cntr++, 0);
        rightLayout->addWidget(limitsGroup, cntr++, 0);
        rightLayout->addWidget(visOptionsGroup, cntr++, 0);
        rightLayout->addWidget(screenshotBtn, cntr++, 0);
        rightLayout->addWidget(saveBtn, cntr++, 0);

        // Center
        QHBoxLayout* centralLayout = new QHBoxLayout;
        centralLayout->addWidget(m_cdtWidget);
        centralLayout->addLayout(rightLayout);
        setLayout(centralLayout);

        setWindowTitle(tr("CDT Visualizer"));

        // Read files list
        updateFilesList();
    }
public slots:
    void updateFilesList()
    {
        // Read files list
        QDir dir = QDir(QString(), tr("*.txt"));
        QFileInfoList list = dir.entryInfoList();
        m_filesList->clear();
        QFileInfoList::iterator it;
        for(it = list.begin(); it != list.end(); ++it)
            m_filesList->addItem(it->fileName());
        m_filesList->setCurrentRow(0);
    }

private:
    CDTWidget* m_cdtWidget;
    QListWidget* m_filesList;
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return QApplication::exec();
}

#include "main.moc"
