#include "CDT.h"

#include <QApplication>
#include <QFileDialog>
#include <QHBoxLayout>

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
        return QSize(600, 600);
    }

private:
    void timerEvent(QTimerEvent* /*e*/)
    {
        update();
    }
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = Q_NULLPTR)
        : QWidget(parent)
    {
        m_cdtWidget = new CDTWidget();

        QHBoxLayout* centralLayout = new QHBoxLayout;
        centralLayout->addWidget(m_cdtWidget);
        setLayout(centralLayout);

        setWindowTitle(tr("CDT Visualizer"));
        layout()->setSizeConstraint(QLayout::SetFixedSize);
    }

private:
    CDTWidget* m_cdtWidget;
};

int main(int argc, char* argv[])
{
    typedef CDT::Triangulation<double> Triangulation;
    typedef CDT::V2d<double> V2d;
    Triangulation cdt;
    {
        std::vector<V2d> vertices;
        vertices.push_back(V2d::make(0, 0));
        vertices.push_back(V2d::make(1, 0));
        vertices.push_back(V2d::make(2, 1));
        vertices.push_back(V2d::make(1, 1));
        cdt.insertVertices(vertices);
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

    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

#include "main.moc"
