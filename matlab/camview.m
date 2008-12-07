function x = camview(u)
    p = u(1:6);
    output = u(7);
    output = 0;
    if output == 1
        subplot(1, 2, 1)
        plot(p(2), p(1), '.b')
        hold on
        plot(p(4), p(3), '.r');
        plot(p(6), p(5), '.k');
        plot([p(2) p(4) p(6)], [p(1) p(3) p(5)],  '-b')
        hold off
        axis([-0.8, 0.8, -0.8, 0.8]);
        title('Camera View');
        grid on;
        xlabel('Y');
        ylabel('X');
        
        subplot(1, 2, 2)
        compass( p(6)-p(2), p(5)-p(1));
    end
    x = 1;
end