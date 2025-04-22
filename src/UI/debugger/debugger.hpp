<QApplication>
#include <QPushButton>
#include <QWidget>
#include <QLabel>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    QWidget window;
    window.setWindowTitle("Hello World App");

    QLabel label("Hi there!", &window);
    label.setAlignment(Qt::AlignCenter);

    window.resize(400, 300);
    window.show();

    return app.exec();
}